/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x80000
  RAM (rwx) :  ORIGIN = 0x20000000, LENGTH = 0x10000
}


SECTIONS
{
    .ate_fixed_block ORIGIN(RAM) (NOLOAD):
    {
        KEEP(*(.ate_fixed)) /* keep my variable even if not referenced */
    } > RAM
}

INCLUDE "nrf52_common.ld"
