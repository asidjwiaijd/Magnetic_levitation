################################################################################
# MRS Version: 2.4.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/apptick.c \
../User/ch32v20x_it.c \
../User/coil.c \
../User/levitation.c \
../User/main.c \
../User/referee.c \
../User/system_ch32v20x.c \
../User/tmag5273.c \
../User/tuning.c \
../User/ui.c \
../User/ws2812.c 

C_DEPS += \
./User/apptick.d \
./User/ch32v20x_it.d \
./User/coil.d \
./User/levitation.d \
./User/main.d \
./User/referee.d \
./User/system_ch32v20x.d \
./User/tmag5273.d \
./User/tuning.d \
./User/ui.d \
./User/ws2812.d 

OBJS += \
./User/apptick.o \
./User/ch32v20x_it.o \
./User/coil.o \
./User/levitation.o \
./User/main.o \
./User/referee.o \
./User/system_ch32v20x.o \
./User/tmag5273.o \
./User/tuning.o \
./User/ui.o \
./User/ws2812.o 

DIR_OBJS += \
./User/*.o \

DIR_DEPS += \
./User/*.d \

DIR_EXPANDS += \
./User/*.234r.expand \


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/cxmnbt/Desktop/Magnetic_levitation/CH32V203G6U/Debug" -I"c:/Users/cxmnbt/Desktop/Magnetic_levitation/CH32V203G6U/Core" -I"c:/Users/cxmnbt/Desktop/Magnetic_levitation/CH32V203G6U/User" -I"c:/Users/cxmnbt/Desktop/Magnetic_levitation/CH32V203G6U/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

