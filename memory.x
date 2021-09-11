MEMORY
{
  /* Bootloader is split in 2 parts: the first part lives in the range 
     0..0x1000; the second part lives at the end of the 1 MB Flash. The range
     selected here collides with neither */ 
  FLASH : ORIGIN = 0x1000, LENGTH = 0x7f000

  /* The bootloader uses the first 8 bytes of RAM to preserve state so don't
     touch them */
  RAM   : ORIGIN = 0x20000008, LENGTH = 0x3fff8
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* You may want to use this variable to locate the call stack and static
   variables in different memory regions. Below is shown the default value */
/* _stack_start = ORIGIN(RAM) + LENGTH(RAM); */

/* You can use this symbol to customize the location of the .text section */
/* If omitted the .text section will be placed right after the .vector_table
   section */
/* This is required only on microcontrollers that store some configuration right
   after the vector table */
/* _stext = ORIGIN(FLASH) + 0x400; */

/* Example of putting non-initialized variables into custom RAM locations. */
/* This assumes you have defined a region RAM2 above, and in the Rust
   sources added the attribute `#[link_section = ".ram2bss"]` to the data
   you want to place there. */
/* Note that the section will not be zero-initialized by the runtime! */
/* SECTIONS {
     .ram2bss (NOLOAD) : ALIGN(4) {
       *(.ram2bss);
       . = ALIGN(4);
     } > RAM2
   } INSERT AFTER .bss;
*/
