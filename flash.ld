/*
	STM32F4XX arm-none-eabi-g++ Project Template (ROM load)
	Linker script
	(C) 2023 Kumohakase
	You are free to destribute, modify without modifying this section.
	Please consider support me on kofi.com https://ko-fi.com/kumohakase
*/

MEMORY {
	/* Adjusted for STM32F4discovery (STM32F407)
		You need to modify for if you will use different one
		that have different memory map */
	FLASH (rx) : ORIGIN = 0x08000000 , LENGTH = 1024K
	RAM (wx) : ORIGIN = 0x20000000 , LENGTH = 128K
}

SECTIONS {
	__stack = ORIGIN(RAM) + LENGTH(RAM); /* Stack top address */
	.text : {
		*(.vects); /* Vector table */
		*(.text*);
		*(.ARM.exidx.text.*);
		*(.rodata*);
		__sidata = .; /* Start of ROM initial data section */
	} >FLASH
	.data : {
		__sdata = .; /* Start of RAM data section */
		*(.data);
		*(.data.*);
		*(.init_array);
		__edata = .; /* End of RAM data section */
	} >RAM AT> FLASH /* written into ROM, then copied into RAM */
	.bss : {
		__sbss = .; /* Start of Bss section */
		*(.COMMON)
		*(.bss.*);
		*(.bss);
		__ebss = .; /* End of bss section */
	} >RAM
}
