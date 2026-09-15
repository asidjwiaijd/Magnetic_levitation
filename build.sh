#!/usr/bin/env bash
# 命令行构建脚本 —— 不依赖 MounRiver IDE, 也不需要 make。
#
# MRS 生成的 obj/makefile 需要 make, 而 MRS2 没有随包提供 make.exe。
# 这个脚本用同一套编译/链接参数直接调用工具链, 产物与 IDE 构建一致。
# IDE 构建仍然可用, 两者互不影响 (本脚本输出到 build/ 而不是 obj/)。
#
# 用法:  ./build.sh          编译 + 链接 + 生成 hex/lst + 打印 size
#        ./build.sh clean    清理

set -e

ROOT="$(cd "$(dirname "$0")" && pwd)"
PROJ="$ROOT/CH32V203G6U"
OUT="$ROOT/build"
NAME=CH32V203G6U

TOOLCHAIN="/d/mounriver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC/bin"
GCC="$TOOLCHAIN/riscv-none-embed-gcc.exe"
OBJCOPY="$TOOLCHAIN/riscv-none-embed-objcopy.exe"
OBJDUMP="$TOOLCHAIN/riscv-none-embed-objdump.exe"
SIZE="$TOOLCHAIN/riscv-none-embed-size.exe"

if [ "$1" = "clean" ]; then
    rm -rf "$OUT"
    echo "cleaned"
    exit 0
fi

if [ ! -x "$GCC" ]; then
    echo "找不到工具链: $GCC" >&2
    echo "MounRiver Studio 装在别的路径时, 改本脚本顶部的 TOOLCHAIN 变量。" >&2
    exit 1
fi

CFLAGS="-march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore \
-fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections \
-fdata-sections -fno-common -Wunused -Wuninitialized -g"

INCLUDES="-I$PROJ/Debug -I$PROJ/Core -I$PROJ/User -I$PROJ/Peripheral/inc"

mkdir -p "$OUT"

SRCS=$(ls "$PROJ"/User/*.c "$PROJ"/Peripheral/src/*.c "$PROJ"/Debug/*.c "$PROJ"/Core/*.c)
OBJS=""

for src in $SRCS; do
    obj="$OUT/$(basename "${src%.c}").o"
    "$GCC" $CFLAGS $INCLUDES -std=gnu99 -c -o "$obj" "$src"
    OBJS="$OBJS $obj"
done

# 只编译 D6 版启动文件 (对应 CH32V203G6U6: 32K flash / 10K RAM)
"$GCC" $CFLAGS $INCLUDES -x assembler-with-cpp -c \
    -o "$OUT/startup.o" "$PROJ/Startup/startup_ch32v20x_D6.S"
OBJS="$OBJS $OUT/startup.o"

"$GCC" $CFLAGS -T "$PROJ/Ld/Link.ld" -nostartfiles -Xlinker --gc-sections \
    -Wl,-Map,"$OUT/$NAME.map" --specs=nano.specs --specs=nosys.specs \
    -o "$OUT/$NAME.elf" $OBJS

"$OBJCOPY" -O ihex "$OUT/$NAME.elf" "$OUT/$NAME.hex"
"$OBJDUMP" --all-headers --demangle --disassemble -M xw "$OUT/$NAME.elf" > "$OUT/$NAME.lst"

echo
"$SIZE" --format=berkeley "$OUT/$NAME.elf"
echo
echo "产物: $OUT/$NAME.elf / .hex"
