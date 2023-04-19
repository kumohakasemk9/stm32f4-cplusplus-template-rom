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
	__stack = ORIGIN(RAM) + LENGTH(RAM);
	.text : {
		*(.vects);
		*(.text*);
		*(.ARM.exidx.text.*);
		*(.init_array);
		*(.rodata*);
		__sidata = .;
	} >FLASH
	.data : {
		__sdata = .;
		*(.data);
		*(.data.*);
		*(.init_array);
		__edata = .;
	} >RAM AT> FLASH
	.bss : {
		__sbss = .;
		*(.COMMON)
		*(.bss.*);
		*(.bss);
		__ebss = .;
	} >RAM
}
