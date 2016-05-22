

MEMORY
{
    /* Bootloader flash 4 KB */
    boot (RX) : ORIGIN = 0x00000000, LENGTH = 0x1000

    /* Flash Size 128 KB minus bootloader sector (4 KB) and CCA area below (88 bytes) */
    rom (RX) : ORIGIN = 0x00001000, LENGTH = 0x1F7A8  /* (was LENGTH = 0x1FFA8 without bootloader) */

    /*
     * Customer Configuration Area and Bootloader Backdoor configuration
     * in flash, up to 88 bytes
     */
    rom_ccfg (RX) : ORIGIN = 0x0001FFA8, LENGTH = 88

    /* RAM Size 20KB */
    ram (RWX) : ORIGIN = 0x20000000, LENGTH = 0x00005000

    /* Application can use GPRAM region as RAM if cache is disabled in CCFG */
    ccmram (RWX) : ORIGIN = 0x11000000, LENGTH = 0x00002000
}

/*. Highest address of the stack. Used in startup file .*/
_estack = ORIGIN(ram) + LENGTH(ram); /* End of SRAM */

INCLUDE cortexm_base.ld

SECTIONS
{
    .boot :
    {
        KEEP(*(.boot_vectors .boot_vectors.*))
    } > boot

    .ccfg :
    {
        KEEP(*(.ccfg))
    } > rom_ccfg
}

