MEMORY {
    FLASH : ORIGIN = 0x10000000, LENGTH = 256

    RAM : ORIGIN = 0x20000000, LENGTH = 264K
}

ENTRY(Reset);

SECTIONS {
    /* The ROM doesn't actually give us a choice in this, but, hey. */
    PROVIDE(_stack_start = ORIGIN(RAM) + LENGTH(RAM));
    PROVIDE(_stext = ORIGIN(FLASH));

    .text _stext : {
        *(.Reset .Reset.*);
        *(.text .text.*);
        . = ALIGN(4);
        __etext = .;
    } >FLASH

    .rodata __etext : ALIGN(4) {
        *(.rodata .rodata.*);
        . = ALIGN(4);
        __erodata = .;
    } >FLASH

    /*
     * These RAM sections are here to detect anything being errantly placed
     * in them. Because we boot without an r0 we can't actually use RAM.
     */
    .data : {
        __sdata = .;
        *(.data .data.*);
        __edata = .;
    } >RAM AT>FLASH

    .bss (NOLOAD) : {
        __sbss = .;
        *(.bss .bss.*);
        __ebss = .;
    } >RAM

    /* Detect accidental got usage too. */
    .got (NOLOAD) : {
        KEEP(*(.got .got.*));
    }

    /DISCARD/ : {
        *(.ARM.exidx);
        *(.ARM.exidx.*);
        *(.ARM.extab.*);
    }
}

ASSERT(__sbss == __ebss, "MUST NOT USE RAM");
ASSERT(__sdata == __edata, "MUST NOT USE RAM");
