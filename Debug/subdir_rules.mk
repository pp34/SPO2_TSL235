################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
main.obj: ../main.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"F:/ti/ccsv6/tools/compiler/ti-cgt-msp430_15.12.1.LTS/bin/cl430" -vmspx --data_model=restricted --use_hw_mpy=F5 --include_path="C:/Users/PP/Desktop/AFE4400beta/titest/driverlib/MSP430F5xx_6xx" --include_path="C:/Users/PP/Desktop/AFE4400beta/titest/GrLib/grlib" --include_path="C:/Users/PP/Desktop/AFE4400beta/titest/GrLib/fonts" --include_path="C:/Users/PP/Desktop/AFE4400beta/titest/LcdDriver" --include_path="C:/Users/PP/Desktop/AFE4400beta/titest/HAL" --include_path="C:/Users/PP/Desktop/AFE4400beta/titest/driverlib/MSP430F5xx_6xx/inc" --include_path="F:/ti/ccsv6/ccs_base/msp430/include" --include_path="C:/Users/PP/Desktop/AFE4400beta/titest/HAL" --include_path="C:/Users/PP/Desktop/AFE4400beta/titest/ADC" --include_path="C:/Users/PP/Desktop/AFE4400beta/titest/DMA" --include_path="C:/Users/PP/Desktop/AFE4400beta/titest/CLK" --include_path="C:/Users/PP/Desktop/AFE4400beta/titest/TIMERA" --include_path="C:/Users/PP/Desktop/AFE4400beta/titest/UART" --include_path="F:/ti/ccsv6/tools/compiler/ti-cgt-msp430_15.12.1.LTS/include" --advice:power=all -g --define=__MSP430F5529__ --diag_warning=225 --diag_wrap=off --display_error_number --silicon_errata=CPU21 --silicon_errata=CPU22 --silicon_errata=CPU23 --silicon_errata=CPU40 --printf_support=full --preproc_with_compile --preproc_dependency="main.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


