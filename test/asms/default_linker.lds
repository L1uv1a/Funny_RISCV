MEMORY {
	
	mem : ORIGIN = 0x00000000, LENGTH = 1k
}

SECTIONS {
	.memory : {
		. = 0x000000;
		*(.text);
		*(*);
		end = .;
		. = ALIGN(4);
	} > mem
}