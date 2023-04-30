 /*
	STM32F4XX arm-none-eabi-g++ Project Template (ROM load)
	Main routine
	(C) 2023 Kumohakase
	You are free to destribute, modify without modifying this section.
	Please consider support me on kofi.com https://ko-fi.com/kumohakase
*/

#include <stm32f4xx.h>

int main() {
	RCC->AHB1ENR |= 0x8;
	GPIOD->MODER = 0x55000000;
	GPIOD->ODR = 0xf000;
	while(1);
}
