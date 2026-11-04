OUTPUT_ARCH(riscv)
ENTRY(_start)
MEMORY { ram (rwx) : ORIGIN = 0x00000000, LENGTH = 64K }
SECTIONS {
    .text : {
        KEEP(*(.text.start))
        *(.text .text.*)
        *(.rodata .rodata.*)
    } > ram
    .data : {
        . = ALIGN(4);
        __global_pointer$ = . + 0x800;
        *(.sdata .sdata.* .data .data.*)
    } > ram
    .bss (NOLOAD) : {
        . = ALIGN(4);
        __bss_start = .;
        *(.sbss .sbss.* .bss .bss.* COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > ram
    __stack_top = ORIGIN(ram) + LENGTH(ram);
    __stack_bottom = __stack_top - 4096;
    __image_end = __bss_end;
    ASSERT(__image_end <= __stack_bottom, "firmware/global data overlaps reserved 4 KiB stack")
    /DISCARD/ : { *(.comment .note .eh_frame .riscv.attributes) }
}
