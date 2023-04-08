#include <stm32f4xx.h>

int main() {
	RCC->AHB1ENR |= 0x8;
	GPIOD->MODER = 0x55000000;
	GPIOD->ODR = 0xf000;
	while(1);
}
