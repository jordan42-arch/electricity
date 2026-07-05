################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccstheia151/ccs/tools/compiler/ti-cgt-armllvm_4.0.0.LTS

GEN_OPTS__FLAG := @"./device.opt" 
GEN_CMDS__FLAG := -Wl,-l"./device_linker.cmd" 

ORDERED_OBJS += \
"./board.o" \
"./empty.o" \
"./ti_msp_dl_config.o" \
"./startup_mspm0g350x_ticlang.o" \
"./BSP/car_control.o" \
"./BSP/encoder.o" \
"./BSP/grey.o" \
"./BSP/imu.o" \
"./BSP/k210_use.o" \
"./BSP/key.o" \
"./BSP/lcd.o" \
"./BSP/lcd_init.o" \
"./BSP/motor.o" \
"./BSP/task1.o" \
"./BSP/task2.o" \
"./BSP/task3.o" \
"./BSP/task4.o" \
"./BSP/test.o" \
$(GEN_CMDS__FLAG) \
-Wl,-ldevice.cmd.genlibs \
-Wl,-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include BSP/subdir_vars.mk
-include subdir_rules.mk
-include BSP/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
11_spi.out 

EXE_OUTPUTS__QUOTED += \
"11_spi.out" 

BIN_OUTPUTS += \
11_spi.hex 

BIN_OUTPUTS__QUOTED += \
"11_spi.hex" 


# All Target
all: $(OBJS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "11_spi.out" secondary-outputs

# Tool invocations
11_spi.out: $(OBJS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: Arm Linker'
	"C:/ti/ccstheia151/ccs/tools/compiler/ti-cgt-armllvm_4.0.0.LTS/bin/tiarmclang.exe" @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -gdwarf-3 -Wl,-m"11_spi.map" -Wl,--heap_size=0x1000 -Wl,--stack_size=0x1000 -Wl,-i"C:/ti/mspm0_sdk_2_10_00_04/source" -Wl,-i"D:/Competiton/2025final/mspm0g3507_Control/New_v3/lcd_spi_1" -Wl,-i"D:/Competiton/2025final/mspm0g3507_Control/New_v3/lcd_spi_1/Debug/syscfg" -Wl,-i"C:/ti/ccstheia151/ccs/tools/compiler/ti-cgt-armllvm_4.0.0.LTS/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="11_spi_linkInfo.xml" -Wl,--rom_model -o "11_spi.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

11_spi.hex: $(EXE_OUTPUTS)
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: Arm Hex Utility'
	"C:/ti/ccstheia151/ccs/tools/compiler/ti-cgt-armllvm_4.0.0.LTS/bin/tiarmhex.exe" --memwidth=8 --romwidth=8 --diag_wrap=off --intel -o "11_spi.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building secondary target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(BIN_OUTPUTS__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "board.o" "empty.o" "ti_msp_dl_config.o" "startup_mspm0g350x_ticlang.o" "BSP\car_control.o" "BSP\encoder.o" "BSP\grey.o" "BSP\imu.o" "BSP\k210_use.o" "BSP\key.o" "BSP\lcd.o" "BSP\lcd_init.o" "BSP\motor.o" "BSP\task1.o" "BSP\task2.o" "BSP\task3.o" "BSP\task4.o" "BSP\test.o" 
	-$(RM) "board.d" "empty.d" "ti_msp_dl_config.d" "startup_mspm0g350x_ticlang.d" "BSP\car_control.d" "BSP\encoder.d" "BSP\grey.d" "BSP\imu.d" "BSP\k210_use.d" "BSP\key.d" "BSP\lcd.d" "BSP\lcd_init.d" "BSP\motor.d" "BSP\task1.d" "BSP\task2.d" "BSP\task3.d" "BSP\task4.d" "BSP\test.d" 
	-@echo 'Finished clean'
	-@echo ' '

secondary-outputs: $(BIN_OUTPUTS)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

