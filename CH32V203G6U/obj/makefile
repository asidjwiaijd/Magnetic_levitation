################################################################################
# MRS Version: 2.4.0
# Automatically-generated file. Do not edit!
################################################################################
-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/subdir.mk
-include Startup/subdir.mk
-include Peripheral/src/subdir.mk
-include Debug/subdir.mk
-include Core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
CH32V203G6U.hex \

SECONDARY_LIST += \
CH32V203G6U.lst \

SECONDARY_SIZE += \
CH32V203G6U.siz \


# All Target
all: 
	$(MAKE) --no-print-directory main-build 

main-build: CH32V203G6U.elf secondary-outputs

# Tool invocations
CH32V203G6U.elf: $(OBJS) $(USER_OBJS_ESCAPE)
	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -T "c:\Users\cxmnbt\Desktop\Magnetic_levitation\CH32V203G6U\Ld\Link.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"CH32V203G6U.map" --specs=nano.specs --specs=nosys.specs -o "CH32V203G6U.elf" $(OBJS) $(LIBS)
CH32V203G6U.hex: CH32V203G6U.elf
	@	riscv-none-embed-objcopy -O ihex "CH32V203G6U.elf" "CH32V203G6U.hex"
CH32V203G6U.lst: CH32V203G6U.elf
	@	riscv-none-embed-objdump --all-headers --demangle --disassemble -M xw "CH32V203G6U.elf" > "CH32V203G6U.lst"
CH32V203G6U.siz: CH32V203G6U.elf
	riscv-none-embed-size --format=berkeley "CH32V203G6U.elf"

# Other Targets
clean:
	-$(RM) $(DIR_OBJS) $(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE) CH32V203G6U.elf
	-$(RM) $(DIR_EXPANDS) $(CALLGRAPH_DOT)
	-$(RM) $(DIR_DEPS)
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets