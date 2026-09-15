"""
线圈线性度 + 噪声本底自动扫描

目的: 把"盯着曲线眼估"换成统计量。一次跑完回答三个问题:

  1. 执行器在哪个指令值上开始偏离线性 (决定 COIL_CMD_LIMIT 该设多少)
  2. 串扰补偿残差 eps 随指令怎么变 (决定 kp_xy 的上界)
  3. 噪声本底有多大, 以及它是不是线圈通电才有的
     —— 这一条最关键: 横向轴的阻尼只有 D 项一个来源, D 项吃的是【相邻两拍
     读数之差】。所以真正该看的不是读数的标准差, 而是【一阶差分的标准差】,
     脚本把它单独列出来并折算成等效的 kd 噪声输出。

用法:
    python host/sweep.py            # 默认 COM4, A 路
    python host/sweep.py COM5 2     # 指定端口和通道 (0=A 1=B 2=C 3=D)

【必须先关掉 tuner.py】—— 串口同一时间只能被一个进程打开。
【台面上不要放矿石】—— 测的是线圈自己的场。

安全: 固件端有 2s 看门狗, 本脚本每 400ms 续一次; 任何异常退出都会在 finally
里把线圈断电。同时监控温度, 超过 TEMP_ABORT 立刻中止。
"""

import sys
import time
import struct
import statistics

import serial

# ---------------- 协议 (与 tuner.py / User/tuning.h 一致) ----------------
DOWN_HEAD = 0xA5
UP_HEAD = 0x5B
REF_HEAD = 0xBB

CMD_STREAM = 0x04
CMD_COIL_TEST = 0x06
UP_TELEM = 0x01
MAX_BODY = 64

# ---------------- 测试参数 ----------------
PORT_DEFAULT = "COM4"
STREAM_DIV = 4              # 1kHz/4 = 250Hz. 34 字节/帧在 115200 下约 85% 占用
SETTLE_S = 0.6              # 换档后丢弃的时间 (L/R 上升沿 + 铁芯稳定)
MEASURE_S = 1.5             # 每档统计时长
LEVELS = [0, 200, 400, 600, 800, -200, -400, -600, -800]
TEMP_ABORT = 60             # ℃, 超过就中止
KEEPALIVE_S = 0.4

# 换算用的常数 (与 board.h / CLAUDE.md 一致)
COUNTS_PER_MM = 23.0        # r=45mm 处, 1mm 横移 ≈ 23 counts
KD_XY = 0.064               # 当前默认值, 用于折算 D 项噪声
DLPF_XY = 0.40
CONTROL_HZ = 1000.0


def build_frame(cmd, payload=b""):
    body = bytes([cmd]) + payload
    frame = bytes([DOWN_HEAD, len(body)]) + body
    return frame + bytes([sum(frame) & 0xFF])


class Link:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.buf = bytearray()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def send(self, data):
        self.ser.write(data)

    def poll(self):
        """读一次串口, 返回本次拆出来的遥测帧列表。"""
        chunk = self.ser.read(4096)
        if chunk:
            self.buf.extend(chunk)
        out = []
        while True:
            while self.buf and self.buf[0] not in (UP_HEAD, REF_HEAD):
                del self.buf[0]
            if not self.buf:
                return out
            if self.buf[0] == REF_HEAD:                 # 裁判上行帧, 跳过
                if len(self.buf) < 4:
                    return out
                if (sum(self.buf[:3]) & 0xFF) == self.buf[3]:
                    del self.buf[:4]
                else:
                    del self.buf[0]
                continue
            if len(self.buf) < 2:
                return out
            ln = self.buf[1]
            total = ln + 3
            if ln == 0 or ln > MAX_BODY:
                del self.buf[0]
                continue
            if len(self.buf) < total:
                return out
            frame = bytes(self.buf[:total])
            del self.buf[:total]
            if (sum(frame[:-1]) & 0xFF) != frame[-1]:
                continue
            if frame[2] == UP_TELEM:
                d = parse_telem(frame[3:total - 1])
                if d:
                    out.append(d)


def parse_telem(p):
    n = len(p)
    if n < 27:                  # 没有原始磁场字段的旧固件, 这个测试做不了
        return None
    bx, by, bz, h, ht = struct.unpack_from("<5h", p, 2)
    coils = struct.unpack_from("<4h", p, 12)
    temp = struct.unpack_from("<b", p, 20)[0]
    rx, ry, rz = struct.unpack_from("<3h", p, 21)
    return dict(state=p[0], bx=bx, by=by, bz=bz, temp=temp,
                rx=rx, ry=ry, rz=rz, coils=coils,
                srate=struct.unpack_from("<h", p, 28)[0] if n >= 30 else -1)


def stats(v):
    """返回 (均值, 标准差, 峰峰值)。"""
    if len(v) < 2:
        return 0.0, 0.0, 0
    return statistics.fmean(v), statistics.pstdev(v), max(v) - min(v)


def diff_std(v):
    """一阶差分的标准差 —— D 项真正吃到的量。"""
    if len(v) < 3:
        return 0.0
    return statistics.pstdev([b - a for a, b in zip(v, v[1:])])


def collect(link, seconds, keep_cb=None):
    """采集指定时长, 返回帧列表。期间按需续看门狗。"""
    frames = []
    t_end = time.time() + seconds
    t_ka = 0.0
    while time.time() < t_end:
        if keep_cb and time.time() - t_ka > KEEPALIVE_S:
            keep_cb()
            t_ka = time.time()
        frames.extend(link.poll())
        time.sleep(0.005)
    return frames


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else PORT_DEFAULT
    ch = int(sys.argv[2]) if len(sys.argv) > 2 else 0

    print(f"端口 {port}   通道 {'ABCD'[ch]}   遥测 {CONTROL_HZ/STREAM_DIV:.0f}Hz")
    print("台面上不要放矿石。开始...\n")

    try:
        link = Link(port)
    except serial.SerialException as e:
        print(f"打不开 {port}: {e}")
        print("→ tuner.py 还开着的话先关掉, 串口同一时间只能被一个进程打开。")
        return 1

    results = []
    try:
        link.send(build_frame(CMD_STREAM, bytes([STREAM_DIV])))
        time.sleep(0.3)
        link.poll()

        # 先确认遥测真的在来
        probe = collect(link, 1.0)
        if not probe:
            print("收不到遥测帧。检查: 固件是不是新的 / 接线 / 波特率")
            return 1
        print(f"遥测正常, 1s 收到 {len(probe)} 帧, 温度 {probe[-1]['temp']}℃\n")

        for lvl in LEVELS:
            def ka(lvl=lvl):
                link.send(build_frame(
                    CMD_COIL_TEST, bytes([ch]) + struct.pack("<h", lvl)))

            ka()
            collect(link, SETTLE_S, ka)                 # 丢弃上升沿
            fr = collect(link, MEASURE_S, ka)
            if not fr:
                print(f"  指令 {lvl:+5d}: 没收到帧, 跳过")
                continue

            t = max(f["temp"] for f in fr)
            if t > TEMP_ABORT:
                print(f"\n温度 {t}℃ 超过 {TEMP_ABORT}℃, 中止")
                break

            row = dict(cmd=lvl, n=len(fr), temp=t,
                       srate=fr[-1]["srate"], state=fr[-1]["state"])
            for axis in ("rx", "ry", "rz", "bx", "by", "bz"):
                v = [f[axis] for f in fr]
                m, sd, pp = stats(v)
                row[axis] = dict(mean=m, sd=sd, pp=pp, dsd=diff_std(v))
            results.append(row)
            print(f"  指令 {lvl:+5d}: raw=({row['rx']['mean']:7.1f},"
                  f"{row['ry']['mean']:7.1f},{row['rz']['mean']:7.1f})  "
                  f"补偿后=({row['bx']['mean']:6.1f},{row['by']['mean']:6.1f},"
                  f"{row['bz']['mean']:6.1f})  {t}℃  {len(fr)}帧")

    except KeyboardInterrupt:
        print("\n中断")
    finally:
        # 无论如何先断电, 发三次防丢帧
        for _ in range(3):
            link.send(build_frame(CMD_COIL_TEST, bytes([ch]) + struct.pack("<h", 0)))
            time.sleep(0.05)
        link.send(build_frame(CMD_STREAM, bytes([0])))      # 关遥测
        time.sleep(0.2)
        link.close()

    report(results, ch)
    return 0


def report(rows, ch):
    if not rows:
        return
    base = next((r for r in rows if r["cmd"] == 0), None)

    print("\n" + "=" * 78)
    print("【1】线性度 —— raw 三轴的 L1 范数 vs 指令")
    print("=" * 78)
    print(f"{'指令':>6} {'L1(raw)':>10} {'L1/1000指令':>12} {'相对 400 档':>12}")
    ref = None
    for r in rows:
        if r["cmd"] == 0:
            continue
        b = base or {a: {"mean": 0.0} for a in ("rx", "ry", "rz")}
        l1 = sum(abs(r[a]["mean"] - b[a]["mean"]) for a in ("rx", "ry", "rz"))
        slope = l1 / abs(r["cmd"]) * 1000
        if abs(r["cmd"]) == 400:
            ref = ref or slope
        rel = f"{slope/ref*100:8.1f}%" if ref else "     —"
        print(f"{r['cmd']:>6} {l1:>10.1f} {slope:>12.1f} {rel:>12}")
    print("  斜率若随指令下降 = 执行器压缩(铁芯饱和/限流)。")
    print("  COIL_CMD_LIMIT 应设在斜率掉到 ~90% 的那一档。")

    print("\n" + "=" * 78)
    print("【2】串扰补偿残差 eps —— 补偿后的 L1 / 原始的 L1")
    print("=" * 78)
    print(f"{'指令':>6} {'L1(补偿后)':>12} {'L1(raw增量)':>12} {'eps':>8} {'kp 上界':>10}")
    for r in rows:
        if r["cmd"] == 0:
            continue
        b = base or {a: {"mean": 0.0} for a in ("rx", "ry", "rz", "bx", "by", "bz")}
        raw = sum(abs(r[a]["mean"] - b[a]["mean"]) for a in ("rx", "ry", "rz"))
        comp = sum(abs(r[a]["mean"]) for a in ("bx", "by", "bz"))
        eps = comp / raw if raw > 1 else 0.0
        # bx_读 = bx_真/(1 - eps*k_c*kp), k_c≈0.6 -> kp 上界
        lim = f"{1.0/(eps*0.6):.1f}" if eps > 0.01 else "∞"
        print(f"{r['cmd']:>6} {comp:>12.1f} {raw:>12.1f} {eps:>8.1%} {lim:>10}")
    print("  kp 上界低于 3 就说明整定窗口是空的(下界实测在 kp≈3)。")

    print("\n" + "=" * 78)
    print("【3】噪声本底 —— 这一项决定 kd 能不能加上去")
    print("=" * 78)
    print(f"{'指令':>6} {'轴':>4} {'标准差':>8} {'峰峰':>7} {'≈mm':>7} "
          f"{'差分σ':>8} {'D项噪声':>9}")
    for r in rows:
        for axis in ("rx", "ry"):
            s = r[axis]
            mm = s["sd"] / COUNTS_PER_MM
            # D 项: d = Δ*CONTROL_HZ, 过 alpha 低通后噪声按 sqrt(a/(2-a)) 缩减
            atten = (DLPF_XY / (2 - DLPF_XY)) ** 0.5
            dnoise = s["dsd"] * CONTROL_HZ * atten * KD_XY
            print(f"{r['cmd']:>6} {axis:>4} {s['sd']:>8.1f} {s['pp']:>7.0f} "
                  f"{mm:>7.2f} {s['dsd']:>8.1f} {dnoise:>9.0f}")
    print(f"  「D项噪声」= kd_xy({KD_XY}) 当前值下, 纯噪声贡献的线圈指令幅度。")
    print("  和限幅 850 比: 超过一两百就说明 D 项被噪声主导, 加 kd = 加噪声。")
    print("  标准差随指令变大 = PWM 开关噪声耦合; 不变 = 传感器/环境本底。")

    if base:
        print(f"\n零指令本底: rx σ={base['rx']['sd']:.1f}  ry σ={base['ry']['sd']:.1f}"
              f"  rz σ={base['rz']['sd']:.1f} counts")
    print(f"传感器更新率 {rows[-1]['srate']} Hz   (1000 = 跟得上控制环)")


if __name__ == "__main__":
    sys.exit(main())
