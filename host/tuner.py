"""
磁悬浮底座调参上位机

复用裁判系统那条 UART (H6 排针, 115200 8N1)。调试帧用 0xA5/0x5B 帧头, 与裁判
协议的 0xAA/0xBB 互不冲突; 遥测默认关闭, 比赛时线上不会多出任何字节。

依赖: PyQt5, pyserial, pyqtgraph(可选, 没装就只显示数值不画图)
"""

import sys
import struct
import time
from collections import deque

import serial
import serial.tools.list_ports
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QColor, QFont
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QComboBox, QSpinBox, QDoubleSpinBox, QTableWidget,
    QTableWidgetItem, QGroupBox, QSlider, QStatusBar, QHeaderView, QMessageBox,
    QCheckBox,
)

try:
    import pyqtgraph as pg
    import numpy as np
    HAS_PG = True
except ImportError:
    HAS_PG = False

# ==================== 协议 (与 User/tuning.h 保持一致) ====================
TUNE_DOWN_HEAD = 0xA5
TUNE_UP_HEAD = 0x5B
REF_UP_HEAD = 0xBB          # 裁判上行帧, 定长 4 字节, 解析时跳过

CMD_PARAM_SET = 0x01
CMD_PARAM_GET = 0x02
CMD_PARAM_LIST = 0x03
CMD_STREAM = 0x04
CMD_ACTION = 0x05
CMD_COIL_TEST = 0x06
CMD_GET_CAL = 0x07

ACT_RECALIB = 0x01
ACT_CAL_HEIGHT = 0x02
ACT_CLEAR_FAULT = 0x03
ACT_SET_HEIGHT = 0x04
ACT_CAL_CT = 0x05
ACT_CAL_TILT = 0x06
ACT_ZERO_HERE = 0x07
ACT_BAL_COILS = 0x08

UP_TELEM = 0x01
UP_PARAM = 0x02
UP_PARAM_INFO = 0x03
UP_ACK = 0x04
UP_CALMAG = 0x05

STATE_NAMES = ["自检", "标定中", "等待矿石", "悬浮中", "故障"]
STATE_COLORS = ["#888888", "#3d8bfd", "#d9a441", "#3fb950", "#f05050"]
COIL_NAMES = ["A", "B", "C", "D"]

PLOT_LEN = 600              # 保留的采样点数
XY_TRAIL = 200              # XY 轨迹图保留的点数
# 帧体(LEN 字段)的上限。必须大于固件端最大的一帧, 否则新增字段后整帧被丢弃
# 且毫无提示 —— 当前最大是遥测帧的 28。
MAX_FRAME_BODY = 64

# 与 board.h 的 ORE_PRESENT_MIN_BZ 一致。那边现在按高度推导(7.5cm), 这里跟着算:
# KZ_DEF / (750 + H_SENSOR_OFFSET)^3 = 6.30421e10 / 780^3
ORE_PRESENT_MIN_BZ = 133
LIMIT_01MM = 100            # 赛规的横向窗口 ±1cm

# 必须与 board.h 的 COIL_CMD_LIMIT 一致。曾经这里硬编码 850 而固件已经砍到 600,
# 结果是"线圈饱和"的告警【永远不会出现】—— 两路顶在 ±600 上跑了几十分钟都没人
# 提醒。改这个值时两边一起改。
COIL_CMD_LIMIT = 600


def ore_offset(bx, by, bz, h):
    """由磁场反算矿石横向偏移, 返回 (x, y) 单位 0.1mm; 不可信时返回 None。

    轴向偶极子:  Bx = 3k·h·x/r^5,  Bz = k(2h^2 - x^2 - y^2)/r^5
    小偏移下 r≈h, 两式相除正好把未知的磁矩 k 约掉:  x = (2/3)·h·Bx/Bz
    —— 所以换磁铁不用重标, 磁铁的极性方向也会自动抵消(分子分母同时变号)。

    两个前提必须记住:
      1. h 来自固件的 kz 换算。kz 没标定时 x 的绝对刻度跟着错, 但方向和相对
         变化仍然可信 —— 判极性、看振荡够用, 读"偏了几毫米"不够。
      2. 倾斜和横移在单颗传感器上简并(CLAUDE.md), 这里给的是"等效横移", 不是
         真实位置。h=3cm 时 5.7° 的倾斜会显示成 1mm 偏移。
    """
    if abs(bz) < ORE_PRESENT_MIN_BZ or h <= 0:
        return None
    k = (2.0 / 3.0) * h / bz
    return k * bx, k * by


def dominant_freq(samples, times):
    """估计一段信号的主频 (Hz)。返回 (频率, 幅值) 或 None。

    调横向环时这是最关键的一个数: 竖直弹跳 √(4g/z) 约 4.5~5.8Hz, 横向不稳定极点
    √(z/2g) 约 3.5Hz, 矿石摇摆模态通常也在 3~8Hz —— 而环路穿越频率要放到 10~20Hz
    才镇得住那个不稳定极点。几档挤在一起, 光看波形分不清是哪一个在发散,
    所以直接把主频算出来。
    """
    n = len(samples)
    if n < 64:
        return None
    dt = times[-1] - times[0]
    if dt <= 0:
        return None
    fs = (n - 1) / dt

    x = np.asarray(samples, dtype=float)
    if not np.all(np.isfinite(x)):
        return None
    x = x - x.mean()
    x = x * np.hanning(n)                   # 不加窗的话谱泄漏会把峰淹掉
    mag = np.abs(np.fft.rfft(x))

    # 跳过前两个 bin: 直流和整段的缓慢漂移, 它们几乎总是最大的
    if len(mag) < 4:
        return None
    k = int(np.argmax(mag[2:])) + 2
    # 幅值折算回原始单位 (加窗损失约 2 倍, 单边谱再 2 倍)
    return k * fs / n, float(mag[k]) * 4.0 / n


def build_frame(cmd, payload=b""):
    body = bytes([cmd]) + payload
    frame = bytes([TUNE_DOWN_HEAD, len(body)]) + body
    return frame + bytes([sum(frame) & 0xFF])


class SerialThread(QThread):
    """后台收帧。只做拆帧和校验, 语义交给主线程。"""

    telem = pyqtSignal(dict)
    param = pyqtSignal(int, float)
    param_info = pyqtSignal(int, str)
    ack = pyqtSignal(int, int)
    calmag = pyqtSignal(dict)
    lost = pyqtSignal(str)

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.ser = None
        self._run = True
        # 收帧统计。"什么都没显示"时靠它区分「没收到字节」「收到但校验错」
        # 「校验对但类型/长度不认识」三种情况。
        self.stats = dict(telem=0, cal=0, ref=0, bad=0, drop=0, unknown=0, bytes=0)

    def stop(self):
        self._run = False
        self.wait(1000)

    def write(self, data):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data)
            except serial.SerialException as e:
                self.lost.emit(str(e))

    def run(self):
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0.05)
        except serial.SerialException as e:
            self.lost.emit(f"打不开 {self.port}: {e}")
            return

        buf = bytearray()
        while self._run:
            try:
                chunk = self.ser.read(512)
            except serial.SerialException as e:
                self.lost.emit(str(e))
                break
            if chunk:
                buf.extend(chunk)
                self.stats["bytes"] += len(chunk)
            self._consume(buf)

        try:
            self.ser.close()
        except Exception:
            pass

    def _consume(self, buf):
        while True:
            # 找帧头, 顺便跳过裁判系统的上行帧
            while buf and buf[0] not in (TUNE_UP_HEAD, REF_UP_HEAD):
                del buf[0]
                self.stats["drop"] += 1
            if not buf:
                return

            if buf[0] == REF_UP_HEAD:
                if len(buf) < 4:
                    return
                # 裁判帧也验校验和。重同步时可能正好落在数据里的 0xBB 上,
                # 不验就会盲删 4 字节, 把后面一帧真帧的头也吃掉。
                if (sum(buf[:3]) & 0xFF) == buf[3]:
                    del buf[:4]
                    self.stats["ref"] += 1
                else:
                    del buf[0]
                    self.stats["drop"] += 1
                continue

            if len(buf) < 2:
                return
            ln = buf[1]
            total = ln + 3                      # head + len + body + cksum
            if ln == 0 or ln > MAX_FRAME_BODY:
                del buf[0]
                self.stats["drop"] += 1
                continue
            if len(buf) < total:
                return

            frame = bytes(buf[:total])
            if (sum(frame[:-1]) & 0xFF) != frame[-1]:
                del buf[0]                      # 校验失败, 往后挪一格重新同步
                self.stats["bad"] += 1
                continue
            del buf[:total]
            self._dispatch(frame[2], frame[3:total - 1])

    def _dispatch(self, typ, payload):
        """按最小长度解析, 多出来的字段当可选。

        固件和上位机的版本不一定同步(比如板子还烧着旧固件), 严格按长度拒收会让
        界面毫无反应又不报错, 极难定位 —— 宁可少显示几个字段也要先跑起来。
        """
        n = len(payload)
        if typ == UP_TELEM and n >= 21:
            st, flags = payload[0], payload[1]
            bx, by, bz, h, ht = struct.unpack_from("<5h", payload, 2)
            coils = struct.unpack_from("<4h", payload, 12)
            temp = struct.unpack_from("<b", payload, 20)[0]
            if n >= 27:                         # 原始磁场是后加的, 旧固件没有
                rx, ry, rz = struct.unpack_from("<3h", payload, 21)
                has_raw = True
            else:
                rx = ry = rz = 0
                has_raw = False
            drops = payload[27] if n >= 28 else 0    # 下行丢帧计数, 更后加的
            srate = struct.unpack_from("<h", payload, 28)[0] if n >= 30 else -1
            # 设定点自整定偏置, 最后加的。收敛后 bx 恒等于 trim, 分不开就没法
            # 判断是矿石真偏了还是 trim 吃掉了传感器偏置。
            if n >= 34:
                tx, ty = struct.unpack_from("<2h", payload, 30)
            else:
                tx = ty = 0
            self.stats["telem"] += 1
            self.telem.emit(dict(
                state=st, settled=bool(flags & 1), test=bool(flags & 2),
                bx=bx, by=by, bz=bz, h=h, ht=ht, coils=coils, temp=temp,
                rx=rx, ry=ry, rz=rz, has_raw=has_raw, drops=drops, srate=srate,
                tx=tx, ty=ty,
            ))
        elif typ == UP_PARAM and n >= 5:
            self.param.emit(payload[0], struct.unpack_from("<f", payload, 1)[0])
        elif typ == UP_PARAM_INFO and n >= 2:
            self.param_info.emit(payload[0], payload[1:].decode("ascii", "replace"))
        elif typ == UP_ACK and n >= 2:
            self.ack.emit(payload[0], payload[1])
        elif typ == UP_CALMAG and n >= 9:
            mp, mn = struct.unpack_from("<2f", payload, 1)
            d = dict(ch=payload[0], mp=mp, mn=mn,
                     sat=payload[9] if n >= 10 else 0,   # 饱和标志是后加的
                     cx=None, cy=None, cz=None)
            if n >= 22:                                  # 分轴串扰是更后加的
                d["cx"], d["cy"], d["cz"] = struct.unpack_from("<3f", payload, 10)
            self.stats["cal"] += 1
            self.calmag.emit(d)
        else:
            self.stats["unknown"] += 1


class Tuner(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("磁悬浮底座 调参上位机")
        self.resize(1280, 800)

        self.ser = None
        self.param_rows = {}            # id -> 表格行
        self.param_names = {}           # id -> 名字
        self.test_ch = -1
        self.test_cmd = 0
        self.last_h = None              # 最近一帧的实测高度, 供「目标←当前高度」用
        self.last_bz = None
        self.ct_xy = {}                 # ch -> (串扰X, 串扰Y), 每 1000 指令
        # 上位机侧独立估计"读数变化率": 统计相邻遥测帧里原始三轴有没有变。
        # 与固件那个数互为交叉验证 —— 固件在 1kHz 拍上数, 这里在遥测拍上数,
        # 上限是遥测频率, 但能直接回答"曲线为什么是台阶状"。
        self.last_raw = None
        self.chg_times = deque(maxlen=400)
        self._last_telem_n = 0
        self._last_stat_t = time.time()

        self.t0 = time.time()
        self.buf = {k: deque(maxlen=PLOT_LEN) for k in
                    ("t", "bx", "by", "bz", "h", "ht", "c0", "c1", "c2", "c3",
                     "px", "py")}
        # XY 轨迹单独存一份短的: 长了糊成一团, 看不出当前在哪
        self.trail = deque(maxlen=XY_TRAIL)

        self._build_ui()

        # 开环测试必须持续续命, 固件端 2s 没收到命令就自动断电
        self.keepalive = QTimer(self)
        self.keepalive.timeout.connect(self._test_keepalive)
        self.keepalive.start(500)

        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self._redraw)
        self.plot_timer.start(33)           # 30 FPS
        self._redraw_n = 0
        self.last_d = None
        self.last_pos = None
        self.last_t = 0.0

        self.stat_timer = QTimer(self)
        self.stat_timer.timeout.connect(self._show_stats)
        self.stat_timer.start(1000)

        self.refresh_ports()

    # ---------------- 界面 ----------------
    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)

        # --- 顶部: 连接与状态 ---
        top = QHBoxLayout()
        self.cb_port = QComboBox()
        self.cb_port.setMinimumWidth(240)
        self.btn_refresh = QPushButton("刷新")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_conn = QPushButton("连接")
        self.btn_conn.clicked.connect(self.toggle_conn)

        self.lb_state = QLabel("未连接")
        f = QFont()
        f.setPointSize(13)
        f.setBold(True)
        self.lb_state.setFont(f)
        self.lb_state.setMinimumWidth(110)
        self.lb_info = QLabel("")

        top.addWidget(QLabel("串口"))
        top.addWidget(self.cb_port)
        top.addWidget(self.btn_refresh)
        top.addWidget(self.btn_conn)
        top.addSpacing(24)
        top.addWidget(self.lb_state)
        top.addWidget(self.lb_info, 1)

        # 传感器 +Z 到底朝上还是朝下, 光看原理图定不下来(封装的 Z 轴方向 + 贴装面
        # 两个因素叠在一起)。而 x = (2/3)h·Bx/Bz 里 Bz 在分母上, Z 轴反了就等于
        # 把 X、Y 同时取反 —— 表现正好是"轨迹两个轴都反了"。留个开关现场翻一下
        # 比推半天靠谱; 定下来之后改 setChecked 的默认值即可。
        self.ck_posinv = QCheckBox("位置取反")
        self.ck_posinv.setToolTip("传感器 +Z 方向不确定时用。Z 轴反号 = X、Y 同时反号。")
        self.ck_posinv.setChecked(True)
        self.ck_posinv.toggled.connect(self._clear_pos)
        top.addWidget(self.ck_posinv)

        top.addWidget(QLabel("遥测分频"))
        self.sp_div = QSpinBox()
        self.sp_div.setRange(0, 100)
        self.sp_div.setValue(10)        # 1kHz / 10 = 100Hz
        self.sp_div.setToolTip("0 = 关闭。控制环 1kHz, 填 10 即 100Hz。")
        self.sp_div.valueChanged.connect(self._send_stream)
        top.addWidget(self.sp_div)
        outer.addLayout(top)

        mid = QHBoxLayout()
        outer.addLayout(mid, 1)

        # --- 左侧: 参数表 + 操作 ---
        left = QVBoxLayout()
        mid.addLayout(left)

        gb_param = QGroupBox("参数 (双击数值可改, 回车下发)")
        lay = QVBoxLayout(gb_param)
        self.tbl = QTableWidget(0, 2)
        self.tbl.setHorizontalHeaderLabels(["名称", "值"])
        self.tbl.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.tbl.horizontalHeader().setSectionResizeMode(1, QHeaderView.Fixed)
        self.tbl.setColumnWidth(1, 130)
        self.tbl.verticalHeader().setVisible(False)
        self.tbl.itemChanged.connect(self._param_edited)
        self.tbl.setMinimumWidth(330)
        lay.addWidget(self.tbl)
        btn_reload = QPushButton("重新读取参数表")
        btn_reload.clicked.connect(self._request_params)
        lay.addWidget(btn_reload)
        left.addWidget(gb_param, 1)

        gb_act = QGroupBox("操作")
        g = QGridLayout(gb_act)
        b_recal = QPushButton("重新标定 (台面须清空)")
        b_recal.clicked.connect(lambda: self._action(ACT_RECALIB, 0))
        b_clear = QPushButton("清除故障")
        b_clear.clicked.connect(lambda: self._action(ACT_CLEAR_FAULT, 0))
        g.addWidget(b_recal, 0, 0, 1, 2)
        g.addWidget(b_clear, 1, 0, 1, 2)

        g.addWidget(QLabel("目标高度 mm"), 2, 0)
        self.sp_target = QSpinBox()
        self.sp_target.setRange(15, 50)
        self.sp_target.setValue(30)
        g.addWidget(self.sp_target, 2, 1)
        b_set = QPushButton("下发目标高度")
        b_set.clicked.connect(lambda: self._action(ACT_SET_HEIGHT, self.sp_target.value()))
        g.addWidget(b_set, 3, 0, 1, 2)

        # 调横向环之前必点。lat_norm = ((h+3)/(h_目标+3))^4, 只有目标等于矿石实际
        # 浮高时系数才是 1; 不对齐的话同一个 kp_xy 会被放大好几倍(最多顶到钳位的
        # 4 倍), 调的根本不是你以为的那个环路。手动对齐太容易忘, 做成一键。
        self.b_align = QPushButton("目标 ← 当前高度  (调横向前必点)")
        self.b_align.setToolTip("把目标高度设成矿石当前实际浮高, 使 lat_norm = 1")
        self.b_align.clicked.connect(self._align_target)
        g.addWidget(self.b_align, 4, 0, 1, 2)

        g.addWidget(QLabel("标定点 0.1mm"), 5, 0)
        self.sp_calh = QSpinBox()
        self.sp_calh.setRange(50, 800)
        self.sp_calh.setValue(300)
        self.sp_calh.setToolTip("把矿石垫在这个已知高度上再点下面的按钮, 反算 kz")
        g.addWidget(self.sp_calh, 5, 1)
        b_calh = QPushButton("按当前高度标定 kz")
        b_calh.clicked.connect(lambda: self._action(ACT_CAL_HEIGHT, self.sp_calh.value()))
        g.addWidget(b_calh, 6, 0, 1, 2)
        left.addWidget(gb_act)

        # ---- 零点微调 ----
        # 零点歪 = 环路认为的中心和真中心差一个常数, 表现为"总有一个力朝一个方向推"。
        # 自整定积分本来就是干这个的, 但它要求矿石能自由移动且快环抓得住 —— 夹具
        # 场合和快环还没调出来的时候两个都不成立, 只能手动。
        gb_zero = QGroupBox("零点微调 (trim_x / trim_y)")
        zl = QGridLayout(gb_zero)
        zl.addWidget(QLabel("步长"), 0, 0)
        self.sp_zstep = QSpinBox()
        self.sp_zstep.setRange(1, 200)
        self.sp_zstep.setValue(10)
        self.sp_zstep.setToolTip("单位是磁场 counts。r=45mm 处约 23 counts/mm, "
                                 "所以 10 大致是 0.4mm。")
        zl.addWidget(self.sp_zstep, 0, 1, 1, 3)
        # 按钮方向按【轨迹图上看到的方向】走, 不是按参数符号 —— 用户要的是
        # "往我看到的这边推一点", 中间那层符号转换不该由人脑来做。
        # 位置取反勾上时显示的 X 是 -x, 所以步进也要跟着反, 否则按钮和图反向。
        for col, (lbl, ax, sgn) in enumerate([
                ("X −", "x", -1), ("X +", "x", +1),
                ("Y −", "y", -1), ("Y +", "y", +1)]):
            b = QPushButton(lbl)
            b.setToolTip("按轨迹图上看到的方向移动矿石 (已跟随「位置取反」)")
            b.clicked.connect(lambda _, a=ax, s=sgn: self._nudge_zero(a, s))
            zl.addWidget(b, 1, col)
        b_zhere = QPushButton("以当前位置为零点")
        b_zhere.setToolTip("把矿石此刻的读数直接抓成零点 —— 自整定积分的手动版本, "
                           "一步到位而不是 3s 收敛。用手或夹具把矿石摆到你想要它停的"
                           "地方再点。要求处于悬浮中状态。")
        b_zhere.clicked.connect(lambda: self._action(ACT_ZERO_HERE, 0))
        zl.addWidget(b_zhere, 2, 0, 1, 4)
        b_zclr = QPushButton("零点清零")
        b_zclr.clicked.connect(self._clear_zero)
        zl.addWidget(b_zclr, 3, 0, 1, 4)
        self.lb_zero = QLabel("trim = (—, —)")
        zl.addWidget(self.lb_zero, 4, 0, 1, 4)
        left.addWidget(gb_zero)

        gb_cal = QGroupBox("标定读数 (每 1000 指令的场强)")
        cl = QGridLayout(gb_cal)
        cl.addWidget(QLabel("路"), 0, 0)
        cl.addWidget(QLabel("正向"), 0, 1)
        cl.addWidget(QLabel("反向"), 0, 2)
        cl.addWidget(QLabel("判读"), 0, 3)
        self.cal_lbl = {}
        for i, name in enumerate(COIL_NAMES):
            cl.addWidget(QLabel(name), i + 1, 0)
            lp, ln, lv = QLabel("—"), QLabel("—"), QLabel("")
            cl.addWidget(lp, i + 1, 1)
            cl.addWidget(ln, i + 1, 2)
            cl.addWidget(lv, i + 1, 3)
            self.cal_lbl[i] = (lp, ln, lv)
        # 串扰的 XY 分量折算成"等效横移" —— 这是唯一能直接和 ±1cm 容差比较的单位。
        # L1 范数大不一定要命(可能全在 Z 轴上), XY 分量大才是真的进了横向环。
        self.lb_ct = QLabel("XY 串扰等效横移: —")
        self.lb_ct.setWordWrap(True)
        cl.addWidget(self.lb_ct, 5, 0, 1, 4)
        b_tilt = QPushButton("标传感器倾斜 (矿石须在正中)")
        b_tilt.setToolTip("飞线焊接不可能完全水平, 倾斜会把 Bz 漏进 Bx/By: "
                          "5 度就是 150+ counts 的假横移, 且随高度变化, 空载零点扣不掉。"
                          "把矿石用非磁性夹具摆在几何正中再点 —— 那里真实 Bx/By 应为 0。")
        b_tilt.clicked.connect(lambda: self._action(ACT_CAL_TILT, 0))
        cl.addWidget(b_tilt, 6, 0, 1, 4)
        b_ct = QPushButton("带矿石重测串扰 (矿石须夹住)")
        b_ct.setToolTip("保留零点, 只重测串扰斜率。铁芯的磁化状态取决于总场, "
                        "空载标的斜率在矿石在场时并不成立。"
                        "标定期间四路会轮流通 ±400, 矿石必须用非磁性夹具固定在工作高度上。")
        b_ct.clicked.connect(lambda: self._action(ACT_CAL_CT, 0))
        cl.addWidget(b_ct, 7, 0, 1, 4)
        b_cal = QPushButton("读取标定结果")
        b_cal.clicked.connect(lambda: self._send(build_frame(CMD_GET_CAL)))
        cl.addWidget(b_cal, 8, 0, 1, 4)
        b_bal = QPushButton("按标定均衡四路强度")
        b_bal.setToolTip("每路取 min(正向,反向) 作为有效强度, 按最弱的那路归一, "
                         "结果写进 gain_a..gain_d。\n"
                         "四路不等造成的是 X-Y【交叉耦合】而不是轴不对称 —— "
                         "X/Y 的有效增益都等于 Σgain, 恒等; 真正变的是\n"
                         "Σ gain·mix_x·mix_y, 四路相等时为 0, 单路弱 δ 就有 δ 的耦合。\n"
                         "现象是环路推的方向与误差方向差一个角度: 绕着转, 或朝某个固定方向飞。")
        b_bal.clicked.connect(lambda: self._action(ACT_BAL_COILS, 0))
        cl.addWidget(b_bal, 9, 0, 1, 4)
        left.addWidget(gb_cal)

        gb_test = QGroupBox("开环单路测试 (裸驱动, 不经增益校正)")
        tl = QGridLayout(gb_test)
        self.sl_test = QSlider(Qt.Horizontal)
        self.sl_test.setRange(-COIL_CMD_LIMIT, COIL_CMD_LIMIT)
        self.sl_test.setValue(0)
        self.sl_test.valueChanged.connect(self._test_slider)
        self.lb_test = QLabel("0")
        self.lb_test.setMinimumWidth(48)
        for i, name in enumerate(COIL_NAMES):
            b = QPushButton(name)
            b.setCheckable(True)
            b.clicked.connect(lambda _, ch=i: self._test_select(ch))
            tl.addWidget(b, 0, i)
            setattr(self, f"btn_test{i}", b)
        tl.addWidget(self.sl_test, 1, 0, 1, 3)
        tl.addWidget(self.lb_test, 1, 3)
        b_stop = QPushButton("停止")
        b_stop.clicked.connect(self._test_stop)
        tl.addWidget(b_stop, 2, 0, 1, 4)
        left.addWidget(gb_test)

        # --- 右侧: 图表 ---
        right = QVBoxLayout()
        mid.addLayout(right, 1)

        if HAS_PG:
            # antialias 是 pyqtgraph 里最贵的开关: 11 条 600 点曲线开着它,
            # 单帧重绘要几十毫秒, 直接把刷新率压到个位数。关掉后曲线略糙但流畅。
            pg.setConfigOptions(antialias=False, background="#1b1b1f",
                                foreground="#c8c8c8")
            self.p_field = pg.PlotWidget(title="磁场 Bx / By / Bz  (0.01mT, 已扣零点与串扰)")
            self.p_field.addLegend()
            self.p_field.showGrid(x=True, y=True, alpha=0.25)
            self.c_bx = self.p_field.plot(pen=pg.mkPen("#ff6b6b", width=2), name="Bx (横向 X)")
            self.c_by = self.p_field.plot(pen=pg.mkPen("#4dd4ac", width=2), name="By (横向 Y)")
            self.c_bz = self.p_field.plot(pen=pg.mkPen("#5b9bff", width=2), name="Bz (高度)")

            self.p_h = pg.PlotWidget(title="高度 (0.1mm)")
            self.p_h.addLegend()
            self.p_h.showGrid(x=True, y=True, alpha=0.25)
            self.c_h = self.p_h.plot(pen=pg.mkPen("#ffd166", width=2), name="实测")
            self.c_ht = self.p_h.plot(pen=pg.mkPen("#888888", width=1, style=Qt.DashLine), name="目标")

            self.p_coil = pg.PlotWidget(title="线圈指令 (±1000)")
            self.p_coil.addLegend()
            self.p_coil.showGrid(x=True, y=True, alpha=0.25)
            cols = ["#ff6b6b", "#4dd4ac", "#5b9bff", "#c792ea"]
            self.c_coil = [self.p_coil.plot(pen=pg.mkPen(cols[i], width=2), name=COIL_NAMES[i])
                           for i in range(4)]

            # --- 矿石横向位置。横向是唯一被动不稳定的轴, 出问题一定先在这里看见 ---
            self.p_pos = pg.PlotWidget(title="矿石横向偏移 (0.1mm, 由 Bx·By/Bz 反算; 含倾斜简并)")
            self.p_pos.addLegend()
            self.p_pos.showGrid(x=True, y=True, alpha=0.25)
            self.c_px = self.p_pos.plot(pen=pg.mkPen("#ff6b6b", width=2),
                                        name="X (正 = 台面上向下)", connect="finite")
            self.c_py = self.p_pos.plot(pen=pg.mkPen("#4dd4ac", width=2),
                                        name="Y (正 = 台面上向左)", connect="finite")
            for v in (LIMIT_01MM, -LIMIT_01MM):     # 赛规 ±1cm
                self.p_pos.addLine(y=v, pen=pg.mkPen("#f05050", width=1, style=Qt.DashLine))
            self.p_pos.setYRange(-150, 150)

            # --- XY 平面轨迹。数据始终是传感器坐标, 只把显示翻过来: +X 朝下、+Y 朝左,
            # 这样图就是你低头看台面时的样子, 推哪边点往哪跑不用在脑子里转一次 ---
            self.p_xy = pg.PlotWidget(title="XY 轨迹 (视角同台面)")
            self.p_xy.showGrid(x=True, y=True, alpha=0.25)
            self.p_xy.setAspectLocked(True)
            self.p_xy.invertX(True)
            self.p_xy.invertY(True)
            self.p_xy.setXRange(-150, 150)
            self.p_xy.setYRange(-150, 150)
            self.p_xy.setLabel("bottom", "← +Y")
            self.p_xy.setLabel("left", "+X ↓")
            L = LIMIT_01MM
            self.p_xy.plot([-L, L, L, -L, -L], [-L, -L, L, L, -L],
                           pen=pg.mkPen("#f05050", width=1, style=Qt.DashLine))
            # 四个线圈的方位标出来, 省得每次回去翻 CLAUDE.md 里那张图。坐标给的是
            # 传感器象限, 翻转由 invertX/Y 负责 —— 改视角时这张表不用动。
            for name, (qy, qx) in zip(COIL_NAMES,
                                      [(-1, 1), (1, -1), (-1, -1), (1, 1)]):
                t = pg.TextItem(name, color="#666677", anchor=(0.5, 0.5))
                t.setPos(qy * 130, qx * 130)
                self.p_xy.addItem(t)
            self.c_trail = self.p_xy.plot(pen=pg.mkPen("#ffd166", width=1))
            self.c_now = pg.ScatterPlotItem(size=11, brush=pg.mkBrush("#5b9bff"),
                                            pen=pg.mkPen("#ffffff", width=1))
            self.p_xy.addItem(self.c_now)

            pos_row = QHBoxLayout()
            pos_row.addWidget(self.p_pos, 2)
            pos_row.addWidget(self.p_xy, 1)

            # 不要在这里调 setDownsampling: PlotItem 会把它转发给自己的每一条
            # 曲线, 而 p_xy 上挂着 ScatterPlotItem(当前位置那个点), 它没有这个
            # 方法, 会每帧抛 AttributeError。600 个点本来也用不上降采样。
            for w in (self.p_field, self.p_h, self.p_coil, self.p_pos, self.p_xy):
                w.setMenuEnabled(False)     # 右键菜单每帧都要算一遍范围
                w.hideButtons()

            right.addWidget(self.p_field, 1)
            right.addWidget(self.p_h, 1)
            right.addWidget(self.p_coil, 1)
            right.addLayout(pos_row, 1)
        else:
            right.addWidget(QLabel("未安装 pyqtgraph, 只显示数值。\npip install pyqtgraph"))
            right.addStretch(1)

        self.lb_num = QLabel("—")
        self.lb_num.setFont(QFont("Consolas", 10))
        right.addWidget(self.lb_num)

        self.lb_freq = QLabel("主频 —")
        self.lb_freq.setFont(QFont("Consolas", 11, QFont.Bold))
        right.addWidget(self.lb_freq)

        self.lb_fz = QLabel("竖直弹跳 f_z —")
        self.lb_fz.setFont(QFont("Consolas", 11, QFont.Bold))
        right.addWidget(self.lb_fz)

        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("就绪")

    # ---------------- 串口 ----------------
    def refresh_ports(self):
        self.cb_port.clear()
        best = 0
        for i, p in enumerate(serial.tools.list_ports.comports()):
            self.cb_port.addItem(f"{p.device} — {p.description}", p.device)
            if any(k in (p.description or "") for k in ("CH340", "CP210", "USB-SERIAL", "USB Serial")):
                best = i
        if self.cb_port.count():
            self.cb_port.setCurrentIndex(best)

    def toggle_conn(self):
        if self.ser:
            self._disconnect()
        else:
            port = self.cb_port.currentData()
            if not port:
                return
            self.ser = SerialThread(port)
            self.ser.telem.connect(self._on_telem)
            self.ser.param.connect(self._on_param)
            self.ser.param_info.connect(self._on_param_info)
            self.ser.ack.connect(self._on_ack)
            self.ser.calmag.connect(self._on_calmag)
            self.ser.lost.connect(self._on_lost)
            self.ser.start()
            self.btn_conn.setText("断开")
            self.statusBar().showMessage(f"已连接 {port}")
            QTimer.singleShot(300, self._request_params)
            QTimer.singleShot(500, self._send_stream)

    def _disconnect(self):
        self._test_stop()
        if self.ser:
            self._send(build_frame(CMD_STREAM, bytes([0])))
            self.ser.stop()
            self.ser = None
        self.btn_conn.setText("连接")
        self.lb_state.setText("未连接")
        self.statusBar().showMessage("已断开")

    def _on_lost(self, msg):
        self.statusBar().showMessage(f"串口错误: {msg}")
        self._disconnect()

    def _send(self, frame):
        if self.ser:
            self.ser.write(frame)

    # ---------------- 参数 ----------------
    def _request_params(self):
        self.tbl.blockSignals(True)
        self.tbl.setRowCount(0)
        self.tbl.blockSignals(False)
        self.param_rows.clear()
        self.param_names.clear()
        self._send(build_frame(CMD_PARAM_LIST))

    def _on_param_info(self, pid, name):
        if pid in self.param_rows:
            return
        row = self.tbl.rowCount()
        # 下面会为每个参数发一个 GET。固件送 PARAM_INFO 是一次性连发的, 如果在
        # 这里同步回发, 十几个 GET 就会背靠背怼过去 —— 115200 下一帧只要 434us,
        # 比固件 1ms 的取帧节奏还快。固件端已经改成队列, 这里再错开一次, 两边
        # 都留余量。
        QTimer.singleShot(40 * (row + 1), lambda p=pid: self._send(
            build_frame(CMD_PARAM_GET, bytes([p]))))
        self.tbl.blockSignals(True)
        self.tbl.insertRow(row)
        it_name = QTableWidgetItem(name)
        it_name.setFlags(it_name.flags() & ~Qt.ItemIsEditable)
        self.tbl.setItem(row, 0, it_name)
        self.tbl.setItem(row, 1, QTableWidgetItem("—"))
        self.tbl.blockSignals(False)
        self.param_rows[pid] = row
        self.param_names[pid] = name

    def _on_param(self, pid, value):
        row = self.param_rows.get(pid)
        if row is None:
            return
        self.tbl.blockSignals(True)
        self.tbl.item(row, 1).setText(f"{value:.6g}")
        self.tbl.blockSignals(False)

    def _param_edited(self, item):
        if item.column() != 1:
            return
        pid = next((k for k, v in self.param_rows.items() if v == item.row()), None)
        if pid is None:
            return
        try:
            val = float(item.text())
        except ValueError:
            self._send(build_frame(CMD_PARAM_GET, bytes([pid])))   # 读回来覆盖掉乱输入
            return
        self._send(build_frame(CMD_PARAM_SET, bytes([pid]) + struct.pack("<f", val)))
        self.statusBar().showMessage(f"已下发 {self.param_names.get(pid, pid)} = {val:g}")

    # ---------------- 动作 ----------------
    def _action(self, code, arg):
        self._send(build_frame(CMD_ACTION, bytes([code]) + struct.pack("<h", int(arg))))

    def _align_target(self):
        h = self.last_h                     # 0.1mm
        if h is None or h <= 0:
            self.statusBar().showMessage("矿石不在场, 读不到高度")
            return
        mm = int(round(h / 10.0))
        mm = max(self.sp_target.minimum(), min(self.sp_target.maximum(), mm))
        self.sp_target.setValue(mm)
        self._action(ACT_SET_HEIGHT, mm)
        self.statusBar().showMessage(f"目标已对齐到 {mm} mm, lat_norm 应回到 1")

    def _send_stream(self):
        self._send(build_frame(CMD_STREAM, bytes([self.sp_div.value()])))

    def _on_calmag(self, d):
        ch, mp, mn, sat = d["ch"], d["mp"], d["mn"], d["sat"]
        if d["cx"] is not None:
            self.ct_xy[ch] = (d["cx"], d["cy"])
            self._show_crosstalk()
        lp, ln, lv = self.cal_lbl[ch]
        lp.setText(f"{mp:.0f}" + ("!" if sat & 1 else ""))
        ln.setText(f"{mn:.0f}" + ("!" if sat & 2 else ""))
        big, small = max(mp, mn), min(mp, mn)
        if sat:
            lv.setText("标定时传感器饱和")
            lv.setStyleSheet("color:#f05050")
        elif big < 5:
            lv.setText("线圈/驱动无响应")
            lv.setStyleSheet("color:#f05050")
        elif small < big / 4:
            lv.setText(f"单向死区 {big/max(small,1e-6):.0f}:1")
            lv.setStyleSheet("color:#d9a441")
        else:
            lv.setText(f"正常 {big/max(small,1e-6):.1f}:1")
            lv.setStyleSheet("color:#3fb950")

    def _show_crosstalk(self):
        """把 XY 串扰折算成等效横移 (mm), 用的是 ore_offset 那套同样的换算。

        意义: 「这一路开到满指令时, 传感器读到的横向偏移会假跳多少毫米」。赛规
        容差是 ±10mm, 所以这个数直接可比 —— 补偿之后的【残差】只要有它的百分之
        几十, 就已经吃掉整个位置预算了。
        """
        if self.last_h is None or self.last_bz is None or abs(self.last_bz) < 1:
            self.lb_ct.setText("XY 串扰等效横移: 需要矿石在场才能折算")
            return
        parts, worst = [], 0.0
        for i, name in enumerate(COIL_NAMES):
            v = self.ct_xy.get(i)
            if v is None:
                continue
            # 满指令 1000 时的 XY 串扰幅值 -> 等效横移 (0.1mm) -> mm
            amp = (v[0] ** 2 + v[1] ** 2) ** 0.5
            mm = (2.0 / 3.0) * self.last_h * amp / abs(self.last_bz) / 10.0
            worst = max(worst, mm)
            parts.append(f"{name} {mm:.0f}")
        if not parts:
            return
        self.lb_ct.setText("XY 串扰等效横移(满指令): " + "  ".join(parts) + " mm")
        # 容差是 ±10mm。满指令的串扰本身远大于它很正常(所以才必须补偿),
        # 但它越大, 补偿残差的绝对值也越大, 留给真实位移的余量就越少。
        self.lb_ct.setStyleSheet("color:#f05050" if worst > 100 else
                                 "color:#d9a441" if worst > 30 else "color:#3fb950")

    def _on_ack(self, cmd, result):
        if result:
            self.statusBar().showMessage(f"命令 0x{cmd:02X} 被拒绝")

    # ---------------- 开环测试 ----------------
    def _test_select(self, ch):
        for i in range(4):
            getattr(self, f"btn_test{i}").setChecked(i == ch)
        self.test_ch = ch
        self._test_send()

    def _test_slider(self, v):
        self.lb_test.setText(str(v))
        self.test_cmd = v
        self._test_send()

    def _test_send(self):
        if self.test_ch < 0:
            return
        self._send(build_frame(CMD_COIL_TEST,
                               bytes([self.test_ch]) + struct.pack("<h", self.test_cmd)))

    def _test_keepalive(self):
        # 固件端 2s 看门狗, 这里每 500ms 续一次
        if self.ser and self.test_ch >= 0 and self.test_cmd != 0:
            self._test_send()

    def _test_stop(self):
        self.test_cmd = 0
        self.sl_test.setValue(0)
        if self.test_ch >= 0:
            self._test_send()
        for i in range(4):
            getattr(self, f"btn_test{i}").setChecked(False)
        self.test_ch = -1

    # ---------------- 遥测 ----------------
    def _on_telem(self, d):
        t = time.time() - self.t0
        b = self.buf
        b["t"].append(t)
        b["bx"].append(d["bx"]); b["by"].append(d["by"]); b["bz"].append(d["bz"])
        b["h"].append(d["h"]); b["ht"].append(d["ht"])
        for i in range(4):
            b[f"c{i}"].append(d["coils"][i])

        pos = ore_offset(d["bx"], d["by"], d["bz"], d["h"])
        if pos is not None and self.ck_posinv.isChecked():
            pos = (-pos[0], -pos[1])
        if pos is None:
            # 矿石不在场时比值是纯噪声, 填 NaN 让曲线断开, 而不是画一条假的零线
            b["px"].append(float("nan")); b["py"].append(float("nan"))
            self.trail.clear()
        else:
            b["px"].append(pos[0]); b["py"].append(pos[1])
            self.trail.append(pos)

        raw = (d["rx"], d["ry"], d["rz"])
        if d.get("has_raw"):
            if self.last_raw is not None and raw != self.last_raw:
                self.chg_times.append(t)
            self.last_raw = raw

        self.last_h = d["h"]
        self.last_bz = d["bz"]
        self.last_t = t
        # 文字全部交给重绘定时器。遥测 100Hz 时每帧都 setText 三个 QLabel 会逼着
        # Qt 每秒做 300 次布局+重绘, 那才是"图谱刷新慢"的主要开销 —— 数据接收
        # 反而很便宜。这里只记住最新一帧。
        self.last_d = d
        self.last_pos = pos

    def _update_labels(self):
        """所有文字渲染, 由重绘定时器按 20Hz 调用, 不跟遥测帧率走。"""
        d = self.last_d
        if d is None:
            return
        pos = self.last_pos
        t = self.last_t

        st = d["state"]
        self.lb_state.setText(STATE_NAMES[st] if st < len(STATE_NAMES) else f"? {st}")
        self.lb_state.setStyleSheet(
            f"color:{STATE_COLORS[st] if st < len(STATE_COLORS) else '#fff'}")

        tags = []
        if d["settled"]:
            tags.append("已到位")
        if d["test"]:
            tags.append("开环测试中")
        if d["temp"] >= 60:
            tags.append(f"高温 {d['temp']}℃")
        if d.get("drops"):
            # 命令没生效时, 这个数是区分「根本没发到」和「发到了没效果」的唯一依据
            tags.append(f"下行丢帧 {d['drops']}")

        self.last_h = d["h"]
        self.last_bz = d["bz"]

        # 上位机侧估计: 近 2 秒内原始读数变化了多少次
        while self.chg_times and t - self.chg_times[0] > 2.0:
            self.chg_times.popleft()
        if len(self.chg_times) >= 4:
            span = self.chg_times[-1] - self.chg_times[0]
            if span > 0.2:
                host_rate = (len(self.chg_times) - 1) / span
                fmax = self.sp_div.value() and 1000.0 / self.sp_div.value() or 0
                # 贴着遥测频率就说明每帧都在变, 真实更新率只会更高, 测不到上界
                tags.append(f"读数变化 {host_rate:.0f}/s" +
                            ("(=遥测上限)" if host_rate > 0.9 * fmax else ""))
        # 传感器更新率。远低于 1kHz 意味着微分项在对一串重复值求导 ——
        # 阻尼看着有、实际是脉冲噪声。
        sr = d.get("srate", -1)
        if sr >= 0:
            if sr < 300:
                tags.append(f"⚠ 传感器仅 {sr} Hz  —— 微分项在对重复值求导")
            elif sr < 900:
                tags.append(f"传感器 {sr} Hz")
        # 饱和时环路实际是断开的 —— 比例项不再起作用, 调什么参数都看不出效果
        # 固件的失控保护要【四路同时】顶限幅才断电, 但环路失去权限用不着四路 ——
        # 象限混合下 (u_x,u_y) 指向某条对角线时, 全部输出都落在【一对】线圈上,
        # 那一对饱和就等于那个方向上再也推不动了, 而另一对还在零附近晃, 看着很正常。
        # 所以这里两路就报, 并把是哪几路说出来。
        sat_ch = [n for n, c in zip("ABCD", d["coils"]) if abs(c) >= COIL_CMD_LIMIT]
        nsat = len(sat_ch)
        if nsat >= 2:
            tags.append(f"⚠ 线圈饱和 {''.join(sat_ch)} ({nsat}/4)"
                        f"  —— 该方向已无控制权限; 查 trim 是否卷绕")
        elif nsat:
            tags.append(f"线圈 {sat_ch[0]} 顶限幅")
        self.lb_info.setText("   ".join(tags))
        self.lb_info.setStyleSheet("color:#f05050" if nsat else "")

        # 零点面板。误差 bx-trim 才是环路真正在追的量, 而屏幕上到处显示的都是 bx,
        # 两者在 trim != 0 时完全不同 —— 不把误差摆出来就会反复掉进
        # "矿石明明在中心, 输出为什么不是零"那个坑。
        tx, ty = d.get("tx", 0), d.get("ty", 0)
        self.lb_zero.setText(f"trim = ({tx:+d}, {ty:+d})    "
                             f"误差 = ({d['bx'] - tx:+d}, {d['by'] - ty:+d})")

        if not d.get("has_raw"):
            raw_txt = "原始   B = 旧固件未上报 (请重新烧录)"
        else:
            # 满量程: X/Y 低量程 13300, Z 高量程 26600 (见 board.h 的量程默认值)
            use = max(abs(d['rx']) / 13300, abs(d['ry']) / 13300, abs(d['rz']) / 26600)
            raw_txt = (f"原始   B = ({d['rx']:6d}, {d['ry']:6d}, {d['rz']:6d})    "
                       f"满量程占用 {use*100:.0f}%"
                       + ("   <<< 逼近饱和" if use > 0.9 else ""))
        if pos is None:
            pos_txt = "横向   矿石不在场 (|Bz| 低于阈值), 位置无意义"
        else:
            r = (pos[0] ** 2 + pos[1] ** 2) ** 0.5
            pos_txt = (f"横向   X = {pos[0]/10:+6.1f}  Y = {pos[1]/10:+6.1f} mm   "
                       f"偏心 {r/10:4.1f} mm"
                       + ("   <<< 超出 ±1cm" if r > LIMIT_01MM else ""))
            # 自整定偏置。收敛后 bx 恒等于 trim, 上面那个"横向"读数就再也动不了,
            # 不把 trim 单独显示出来就分不清"矿石真在中心"和"trim 把偏差吃了"。
            # 换算成毫米用的是 ore_offset 那套同样的比值, 所以两个数可以直接比。
            tr = ore_offset(d.get("tx", 0), d.get("ty", 0), d["bz"], d["h"])
            if tr is not None and (d.get("tx") or d.get("ty")):
                if self.ck_posinv.isChecked():
                    tr = (-tr[0], -tr[1])
                tn = (tr[0] ** 2 + tr[1] ** 2) ** 0.5
                pos_txt += (f"      零点 trim = ({d['tx']:+5d},{d['ty']:+5d}) counts"
                            f" ≈ {tn/10:4.1f} mm")
        # 传感器到矿石的真实距离 r = 立方根(kz/|Bz|)。它【不依赖 H_SENSOR_OFFSET】,
        # 所以传感器一旦挪位置(飞线抬高之类), 就靠它来反推新的 offset:
        #   Δ = r_挪之前 - r_挪之后   ->   新 offset = 旧 offset - 10Δ
        # 而 h 是 r 减掉 offset 得来的, offset 没改对的时候 h 是错的, r 仍然对。
        kz = self._param_value("kz")
        if kz and abs(d["bz"]) >= 1:
            r_mm = (kz / abs(d["bz"])) ** (1.0 / 3.0) / 10.0
            r_txt = f"    r(传感器→矿石) = {r_mm:5.1f} mm"
        else:
            r_txt = ""
        self.lb_num.setText(
            f"补偿后 B = ({d['bx']:6d}, {d['by']:6d}, {d['bz']:6d}) ×0.01mT    "
            f"h = {d['h']/10:5.1f} / {d['ht']/10:5.1f} mm" + r_txt + "    "
            f"线圈 = {d['coils']}    温度 = {d['temp']}℃"
            + chr(10) + raw_txt + chr(10) + pos_txt
        )

    def _param_value(self, name):
        """从参数表读回某个参数的当前值; 读不到返回 None。"""
        for pid, nm in self.param_names.items():
            if nm == name:
                row = self.param_rows.get(pid)
                if row is None:
                    return None
                try:
                    return float(self.tbl.item(row, 1).text())
                except (ValueError, AttributeError):
                    return None
        return None

    def _set_param(self, name, val):
        """按名字下发一个参数。名字→ID 的对应来自设备的 PARAM_LIST, 所以上位机
        这边不硬编码 ID —— 固件增删字段时这里不用跟着改。"""
        for pid, nm in self.param_names.items():
            if nm == name:
                self._send(build_frame(CMD_PARAM_SET,
                                       bytes([pid]) + struct.pack("<f", float(val))))
                return True
        self.status.showMessage(f"设备没有参数 {name} (固件是旧的?)", 4000)
        return False

    def _nudge_zero(self, axis, sgn):
        """零点微调。基准取【遥测里的实时值】而不是参数表 —— 参数表只在设备回读
        时才刷新, 自整定开着的时候它一直是过期的, 拿它当基准会把积分的成果抹掉。"""
        if self.last_d is None:
            return
        step = self.sp_zstep.value() * sgn
        # 按钮写的是"轨迹图上看到的方向"。取反勾上时显示的 X 是 -x, 步进跟着反,
        # 否则按钮和图会反向 —— 那是最难自查的一类错。
        if self.ck_posinv.isChecked():
            step = -step
        cur = self.last_d.get("tx" if axis == "x" else "ty", 0)
        self._set_param("trim_x" if axis == "x" else "trim_y", cur + step)

    def _clear_zero(self):
        self._set_param("trim_x", 0.0)
        self._set_param("trim_y", 0.0)

    def _clear_pos(self):
        """翻转取向时把旧点丢掉 —— 缓存里存的是翻转后的结果, 留着会和新点混在
        一起, 看上去像矿石突然跳了一下。"""
        self.trail.clear()
        # 填 NaN 而不是清空: px/py 必须和 t 一样长, 否则 setData 会因长度不等抛异常
        for k in ("px", "py"):
            n = len(self.buf[k])
            self.buf[k].clear()
            self.buf[k].extend([float("nan")] * n)

    def _show_stats(self):
        if not self.ser:
            return
        st = self.ser.stats
        # 实测帧率, 每秒结算。"曲线看着点很稀"到底是收得少还是画得少, 只有它能分清:
        # 这个数是收到的帧数, 与绘图完全无关。
        now = time.time()
        dn = st["telem"] - self._last_telem_n
        dt = now - self._last_stat_t
        self._last_telem_n, self._last_stat_t = st["telem"], now
        hz = dn / dt if dt > 0 else 0
        want = 1000.0 / self.sp_div.value() if self.sp_div.value() else 0

        msg = (f"遥测 {hz:5.1f} Hz (应为 {want:.0f})   收 {st['bytes']}B   "
               f"帧 {st['telem']}   标定 {st['cal']}   "
               f"裁判帧 {st['ref']}   校验错 {st['bad']}   丢字节 {st['drop']}   "
               f"未识别 {st['unknown']}")
        if want and hz < want * 0.7:
            msg += "   <<< 收帧率明显偏低: 查固件主循环有没有丢拍"
        if st["bytes"] == 0:
            msg += "   <<< 一个字节都没收到: 查串口/接线/波特率"
        elif st["telem"] == 0 and self.sp_div.value() > 0:
            msg += "   <<< 收到数据但没有遥测帧: 固件版本可能不匹配"
        self.statusBar().showMessage(msg)

    def _redraw(self):
        self._redraw_n += 1
        self._update_labels()               # 文字 30Hz, 不再跟 100Hz 的遥测走

        if not HAS_PG or not self.buf["t"]:
            return

        # deque -> ndarray 转一次就够, 别在每条曲线上各转一遍
        arr = {k: np.fromiter(v, dtype=float, count=len(v))
               for k, v in self.buf.items()}
        t = arr["t"]
        self.c_bx.setData(t, arr["bx"])
        self.c_by.setData(t, arr["by"])
        self.c_bz.setData(t, arr["bz"])
        self.c_h.setData(t, arr["h"])
        self.c_ht.setData(t, arr["ht"])
        for i in range(4):
            self.c_coil[i].setData(t, arr[f"c{i}"])

        # FFT 三次 600 点, 没必要每帧都算 —— 频率读数 6Hz 刷新绰绰有余
        if self._redraw_n % 5 == 0:
            self._update_freq(t)

        self.c_px.setData(t, arr["px"])
        self.c_py.setData(t, arr["py"])
        if self.trail:
            # 画布是 X 朝上、Y 朝右, 所以横坐标喂 Y、纵坐标喂 X
            tx = [p[1] for p in self.trail]
            ty = [p[0] for p in self.trail]
            self.c_trail.setData(tx, ty)
            self.c_now.setData([tx[-1]], [ty[-1]])
        else:
            self.c_trail.setData([], [])
            self.c_now.setData([], [])

    # 频率区间 -> 成因。边界见 dominant_freq 的说明。
    FREQ_VERDICT = (
        (2.0,   "漂移/积分项, 不是振荡"),
        (9.0,   "★ 刚体模态(弹跳/摇摆) —— 倾斜简并, 调参解决不了"),
        (30.0,  "穿越频率处相位不够 -> 加 kd_xy, dlpf_xy 往大调"),
        (1e9,   "执行器滞后/串扰残差 -> 扫 ct_lag, 减 kd_xy"),
    )

    def _update_freq(self, t):
        fx = dominant_freq(self.buf["bx"], t)
        fy = dominant_freq(self.buf["by"], t)
        fz = dominant_freq(self.buf["bz"], t)
        if fx is None or fy is None:
            self.lb_freq.setText("主频 —  (样本不足)")
            return

        # 取幅值大的那一轴: 两轴通常一起振, 弱的那轴信噪比差, 峰位不可信
        f, amp = fx if fx[1] >= fy[1] else fy
        axis = "Bx" if fx[1] >= fy[1] else "By"

        # Bz 的主频就是竖直弹跳频率 f_z, 它一个数把整个被控对象钉死:
        #   竖直刚度 k_z = m(2*pi*f_z)^2
        #   Earnshaw: k_x = -k_z/2  ->  横向不稳定极点 a = 2*pi*f_z/sqrt(2) = 4.44*f_z
        #   最优阻尼 kd/kp = 0.7/a = 0.158/f_z
        # 测法: 所有增益归零, 轻敲矿石, 读这个数。
        if fz and fz[1] >= 5.0:
            a = 4.44 * fz[0]                    # 2*pi*f_z/sqrt(2)
            txt = (f"竖直弹跳 f_z = {fz[0]:4.1f} Hz   "
                   f"→ 横向极点 {a:5.1f} rad/s ({a/6.283:4.2f} Hz), "
                   f"发散时间常数 {1000/a:4.0f} ms")

            # 有横向振荡时可以把环路增益 G 一并辨识出来:
            #   kd=0 的纯比例反馈 -> s^2 + (G*kp - a^2) = 0 -> w_osc^2 = G*kp - a^2
            #   zeta=0.7 要求 G*kd = 1.4*w_osc
            #   两式相除, G 被约掉:  kd = kp * 1.4*w_osc / (w_osc^2 + a^2)
            # 【只有 kd 设成 0、kp 调到刚好等幅振荡时测出来的 w_osc 才代入得对】
            w = 6.283 * f
            kp = self._param_value("kp_xy")
            if amp >= 5.0 and w > a and kp:
                kd = kp * 1.4 * w / (w * w + a * a)
                txt += f"\n→ 按当前振荡 {f:.1f}Hz 与 kp_xy={kp:g} 反推:  kd_xy = {kd:.4f}"
            self.lb_fz.setText(txt)
            self.lb_fz.setStyleSheet("color:#3fb950")
        else:
            self.lb_fz.setText("竖直弹跳 f_z —  (增益归零后轻敲矿石, 让 Bz 振起来)")
            self.lb_fz.setStyleSheet("color:#888888")

        if amp < 5.0:               # 0.05mT, 基本就是噪声
            self.lb_freq.setText(f"主频 {f:5.1f} Hz ({axis} 幅值 {amp:.0f}, 太小, 不用管)")
            self.lb_freq.setStyleSheet("color:#888888")
            return

        # 遥测速率决定可观测上限。峰贴着 Nyquist 时多半是更高频的东西混叠下来的,
        # 这时判读全错(比如 65Hz 在 100Hz 采样下会显示成 35Hz), 必须先提速再看。
        nyq = (len(t) - 1) / (t[-1] - t[0]) / 2.0
        if f > nyq * 0.8:
            self.lb_freq.setText(
                f"主频 {f:5.1f} Hz  (贴近 Nyquist {nyq:.0f} Hz, 可能是混叠) "
                f"  <<< 把遥测分频调到 3 再看")
            self.lb_freq.setStyleSheet("color:#f05050")
            return

        for hi, verdict in self.FREQ_VERDICT:
            if f < hi:
                break
        self.lb_freq.setText(f"主频 {f:5.1f} Hz  ({axis} 幅值 {amp:4.0f})   {verdict}")
        self.lb_freq.setStyleSheet("color:#d9a441" if f < 9.0 else "color:#c8c8c8")

    def closeEvent(self, ev):
        self._disconnect()
        ev.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = Tuner()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
