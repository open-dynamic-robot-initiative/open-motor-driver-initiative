CLA_SCRATCHPAD_SIZE = 0x100;
--undef_sym=__cla_scratchpad_end
--undef_sym=__cla_scratchpad_start

MEMORY
{
   /* BEGIN is used for the "boot to SARAM" bootloader mode   */
   BEGIN            : origin = 0x000000, length = 0x000002
   BOOT_RSVD        : origin = 0x000002, length = 0x0001AF     /* Part of M0, BOOT rom will use this for stack */
   RAMM0            : origin = 0x0001B1, length = 0x00024F
   RAMM1            : origin = 0x000400, length = 0x0003F8     /* on-chip RAM block M1 */
//   RAMM1_RSVD       : origin = 0x0007F8, length = 0x000008     /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */
   RAMD0            : origin = 0x00C000, length = 0x000800
   RAMD1            : origin = 0x00C800, length = 0x000800
   RAMLS0           : origin = 0x008000, length = 0x000800
   RAMLS1           : origin = 0x008800, length = 0x000800
   RAMLS2           : origin = 0x009000, length = 0x000800
   RAMLS3           : origin = 0x009800, length = 0x000800
   RAMLS4           : origin = 0x00A000, length = 0x000800
   RAMLS5           : origin = 0x00A800, length = 0x000800
   RAMLS6_7         : origin = 0x00B000, length = 0x001000
   //RAMLS6           : origin = 0x00B000, length = 0x000800
   //RAMLS7           : origin = 0x00B800, length = 0x000800
   RAMGS0           : origin = 0x00D000, length = 0x001000
   RAMGS1           : origin = 0x00E000, length = 0x001000
   RAMGS2           : origin = 0x00F000, length = 0x001000
   RAMGS3_4			: origin = 0x010000, length = 0x002000
   //RAMGS4           : origin = 0x011000, length = 0x001000
   RAMGS5           : origin = 0x012000, length = 0x001000
   RAMGS6           : origin = 0x013000, length = 0x001000
   RAMGS7           : origin = 0x014000, length = 0x001000
   RAMGS8           : origin = 0x015000, length = 0x001000
   RAMGS9           : origin = 0x016000, length = 0x001000
   RAMGS10_14       : origin = 0x017000, length = 0x005000
   //RAMGS11          : origin = 0x018000, length = 0x004000
   //RAMGS12          : origin = 0x019000, length = 0x004000
   //RAMGS13          : origin = 0x01A000, length = 0x001000
   //RAMGS14          : origin = 0x01B000, length = 0x001000
   RAMGS15          : origin = 0x01C000, length = 0x000FF8
//   RAMGS15_RSVD     : origin = 0x01CFF8, length = 0x000008     /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

   /* Flash sectors */
   FLASH0           : origin = 0x080000, length = 0x002000	/* on-chip Flash */
   FLASH1           : origin = 0x082000, length = 0x002000	/* on-chip Flash */
   FLASH2           : origin = 0x084000, length = 0x002000	/* on-chip Flash */
   FLASH3           : origin = 0x086000, length = 0x002000	/* on-chip Flash */
   FLASH4           : origin = 0x088000, length = 0x008000	/* on-chip Flash */
   FLASH5           : origin = 0x090000, length = 0x008000	/* on-chip Flash */
   FLASH6           : origin = 0x098000, length = 0x008000	/* on-chip Flash */
   FLASH7           : origin = 0x0A0000, length = 0x008000	/* on-chip Flash */
   FLASH8           : origin = 0x0A8000, length = 0x008000	/* on-chip Flash */
   FLASH9           : origin = 0x0B0000, length = 0x008000	/* on-chip Flash */
   FLASH10          : origin = 0x0B8000, length = 0x002000	/* on-chip Flash */
   FLASH11          : origin = 0x0BA000, length = 0x002000	/* on-chip Flash */
   FLASH12          : origin = 0x0BC000, length = 0x002000	/* on-chip Flash */
   FLASH13          : origin = 0x0BE000, length = 0x002000	/* on-chip Flash */
   CPU1TOCPU2RAM    : origin = 0x03A000, length = 0x000800
   CPU2TOCPU1RAM    : origin = 0x03B000, length = 0x000800

   CPUTOCMRAM       : origin = 0x039000, length = 0x000800
   CMTOCPURAM       : origin = 0x038000, length = 0x000800

   CANA_MSG_RAM     : origin = 0x049000, length = 0x000800
   CANB_MSG_RAM     : origin = 0x04B000, length = 0x000800
   RESET           	: origin = 0x3FFFC0, length = 0x000002

   CLA1_MSGRAMLOW   : origin = 0x001480,   length = 0x000080
   CLA1_MSGRAMHIGH  : origin = 0x001500,   length = 0x000080
}


SECTIONS
{
   codestart        : > BEGIN
   .text            : > RAMGS10_14
   .cinit           : > RAMLS3
   .switch          : > RAMM0
   .reset           : > RESET, TYPE = DSECT /* not used, */

   .stack           : > RAMM1
#if defined(__TI_EABI__)
   .bss             : > RAMLS2
   .bss:output      : > RAMLS3
   .init_array      : > RAMM0
   .const           : > RAMGS3_4
   .data            : > RAMLS6_7
   .sysmem          : > RAMLS4
#else
   .pinit           : > RAMM0
   .ebss            : > RAMLS4
   .econst          : > RAMLS4
   .esysmem         : > RAMLS4
#endif

   ramgs0 : > RAMGS0, type=NOINIT
   ramgs1 : > RAMGS1, type=NOINIT

   MSGRAM_CPU1_TO_CPU2 > CPU1TOCPU2RAM, type=NOINIT
   MSGRAM_CPU2_TO_CPU1 > CPU2TOCPU1RAM, type=NOINIT
   MSGRAM_CPU_TO_CM   > CPUTOCMRAM, type=NOINIT
   MSGRAM_CM_TO_CPU   > CMTOCPURAM, type=NOINIT
   
    /* CLA specific sections */
   Cla1Prog         : > RAMLS5

   ClaData			: > RAMLS0
   CLADataLS1		: > RAMLS1

   Cla1ToCpuMsgRAM  : > CLA1_MSGRAMLOW, type=NOINIT
   CpuToCla1MsgRAM  : > CLA1_MSGRAMHIGH, type=NOINIT
   Cla1DataRam      : >> RAMLS0 | RAMLS1

   /* CLA C compiler sections */
   //
   // Must be allocated to memory the CLA has write access to
   //
   CLAscratch       :
                     { *.obj(CLAscratch)
                     . += CLA_SCRATCHPAD_SIZE;
                     *.obj(CLAscratch_end) } >  RAMLS1

   .scratchpad      : > RAMLS1
   .bss_cla		    : > RAMLS1
   .const_cla	    : > RAMLS1

    .TI.ramfunc : {} >> RAMM0 | RAMD0

}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
