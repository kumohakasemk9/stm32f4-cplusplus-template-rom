
#	STM32F4XX arm-none-eabi-g++ Project Template (ROM load)
#	Makefile
#	(C) 2023 Kumohakase
#	You are free to destribute, modify without modifying this section.
#	Please consider support me on kofi.com https://ko-fi.com/kumohakase

CXX=arm-none-eabi-g++
CPUFLAGS=-mcpu=cortex-m4 -mfpu=fpv4-sp-d16
CPPFLAGS=-g3 -I . -I ~/harddisk_home/programs/CMSIS_5-5.7.0/CMSIS/Core/Include/ -D STM32F40XX $(CPUFLAGS) -fno-exceptions
OBJS=main.o start.o
OUTPUT=output.bin

all: $(OBJS)
	$(CXX) $(CPUFLAGS) -T flash.ld -nostartfiles -o $(OUTPUT) $^
	arm-none-eabi-objdump -hdS $(OUTPUT) > report
	arm-none-eabi-objdump -h $(OUTPUT)

clean:
	rm -f $(OBJS) $(OUTPUT) report
