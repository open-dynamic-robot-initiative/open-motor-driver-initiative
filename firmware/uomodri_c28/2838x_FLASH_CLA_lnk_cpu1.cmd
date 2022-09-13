CLA_SCRATCHPAD_SIZE = 0x100;
--undef_sym=__cla_scratchpad_end
--undef_sym=__cla_scratchpad_start

MEMORY
{
   /* BEGIN is used for the "boot to Flash" bootloader mode   */
   BEGIN            : origin = 0x080000, length = 0x000002
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
   RAMLS6           : origin = 0x00B000, length = 0x000800
   RAMLS7           : origin = 0x00B800, length = 0x000800
   RAMGS0           : origin = 0x00D000, length = 0x001000
   RAMGS1           : origin = 0x00E000, length = 0x001000
   RAMGS2           : origin = 0x00F000, length = 0x001000
   RAMGS3           : origin = 0x010000, length = 0x001000
   RAMGS4           : origin = 0x011000, length = 0x001000
   RAMGS5           : origin = 0x012000, length = 0x001000
   RAMGS6           : origin = 0x013000, length = 0x001000
   RAMGS7           : origin = 0x014000, length = 0x001000
   RAMGS8           : origin = 0x015000, length = 0x001000
   RAMGS9           : origin = 0x016000, length = 0x001000
   RAMGS10          : origin = 0x017000, length = 0x001000
   RAMGS11          : origin = 0x018000, length = 0x001000
   RAMGS12          : origin = 0x019000, length = 0x001000
   RAMGS13          : origin = 0x01A000, length = 0x001000
   RAMGS14          : origin = 0x01B000, length = 0x001000
   RAMGS15          : origin = 0x01C000, length = 0x000FF8
//   RAMGS15_RSVD     : origin = 0x01CFF8, length = 0x000008     /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

   /* Flash sectors */
   FLASH0           : origin = 0x080002, length = 0x001FFE  /* on-chip Flash */
   FLASH1           : origin = 0x082000, length = 0x002000  /* on-chip Flash */
   FLASH2           : origin = 0x084000, length = 0x002000  /* on-chip Flash */
   FLASH3           : origin = 0x086000, length = 0x002000  /* on-chip Flash */
   FLASH4           : origin = 0x088000, length = 0x008000  /* on-chip Flash */
   FLASH5           : origin = 0x090000, length = 0x008000  /* on-chip Flash */
   FLASH6           : origin = 0x098000, length = 0x008000  /* on-chip Flash */
   FLASH7           : origin = 0x0A0000, length = 0x008000  /* on-chip Flash */
   FLASH8           : origin = 0x0A8000, length = 0x008000  /* on-chip Flash */
   FLASH9           : origin = 0x0B0000, length = 0x008000  /* on-chip Flash */
   FLASH10          : origin = 0x0B8000, length = 0x002000  /* on-chip Flash */
   FLASH11          : origin = 0x0BA000, length = 0x002000  /* on-chip Flash */
   FLASH12          : origin = 0x0BC000, length = 0x002000  /* on-chip Flash */
   FLASH13          : origin = 0x0BE000, length = 0x001FF0  /* on-chip Flash */
//   FLASH13_RSVD     : origin = 0x0BFFF0, length = 0x000010  /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

   CPU1TOCPU2RAM   : origin = 0x03A000, length = 0x000800
   CPU2TOCPU1RAM   : origin = 0x03B000, length = 0x000800
   CPUTOCMRAM      : origin = 0x039000, length = 0x000800
   CMTOCPURAM      : origin = 0x038000, length = 0x000800

   CANA_MSG_RAM     : origin = 0x049000, length = 0x000800
   CANB_MSG_RAM     : origin = 0x04B000, length = 0x000800

   RESET            : origin = 0x3FFFC0, length = 0x000002

   CLA1_MSGRAMLOW   : origin = 0x001480,   length = 0x000080
   CLA1_MSGRAMHIGH  : origin = 0x001500,   length = 0x000080
}

SECTIONS
{
   codestart           : > BEGIN, ALIGN(8)
   .text               : > FLASH4, ALIGN(8)
   .cinit              : > FLASH4, ALIGN(8)
   .switch             : > FLASH4, ALIGN(8)
   .reset              : > RESET, TYPE = DSECT /* not used, */
   .stack              : > RAMM1

#if defined(__TI_EABI__)
   .init_array      : > FLASH4, ALIGN(8)
   .bss             : > RAMLS3
   .bss:output      : > RAMLS3
   .bss:cio			: > RAMLS3
   .data            : > RAMLS4
   .sysmem          : > RAMLS4
   /* Initalized sections go in Flash */
   .const           : > FLASH4, ALIGN(8)
#else
   .pinit           : > FLASH1, ALIGN(8)
   .ebss            : > RAMLS3
   .esysmem         : > RAMLS4
   /* Initalized sections go in Flash */
   .econst          : >> FLASH4 | FLASH5, ALIGN(8)
#endif

   ramgs0			: > RAMGS0//, type=NOINIT
   ramgs1			: > RAMGS1, type=NOINIT

   MSGRAM_CPU1_TO_CPU2 : > CPU1TOCPU2RAM, type=NOINIT
   MSGRAM_CPU2_TO_CPU1 : > CPU2TOCPU1RAM, type=NOINIT
   MSGRAM_CPU_TO_CM    : > CPUTOCMRAM//, type=NOINIT
   MSGRAM_CM_TO_CPU    : > CMTOCPURAM//, type=NOINIT

   dclfuncs : > FLASH1, ALIGN(8)

//#if defined(__TI_EABI__)
//    /* CLA specific sections */
//    Cla1Prog        : LOAD = FLASH4,
//                      RUN = RAMLS6,
//                      LOAD_START(Cla1ProgLoadStart),
//                      //LOAD_END(Cla1funcsLoadEnd),
//                      RUN_START(Cla1ProgRunStart),
//                      LOAD_SIZE(Cla1ProgLoadSize),
//                      PAGE = 0, ALIGN(4)
//#else
//    /* CLA specific sections */
//    Cla1Prog        : LOAD = FLASH5,
//                      RUN = RAMLS5,
//                      LOAD_START(_Cla1ProgLoadStart),
//                      RUN_START(_Cla1ProgRunStart),
//                      LOAD_SIZE(_Cla1ProgLoadSize),
//                      PAGE = 0, ALIGN(4)
//#endif

    /* CLA specific sections */
//#if defined(__TI_EABI__)
   Cla1Prog         :   LOAD = FLASH4,
                        RUN = RAMLS2,
                        LOAD_START(Cla1funcsLoadStart),
                        LOAD_END(Cla1funcsLoadEnd),
                        RUN_START(Cla1funcsRunStart),
                        LOAD_SIZE(Cla1funcsLoadSize),
                        ALIGN(8)
//#else
//   Cla1Prog         :   LOAD = FLASH4,
//                        RUN = RAMLS2,
//                        LOAD_START(_Cla1funcsLoadStart),
//                        LOAD_END(_Cla1funcsLoadEnd),
//                        RUN_START(_Cla1funcsRunStart),
//                        LOAD_SIZE(_Cla1funcsLoadSize),
//                        ALIGN(8)
//#endif
//   Cla1Prog         : > RAMLS2
   CLADataLS0       : > RAMLS0
   CLADataLS1       : > RAMLS1

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
   .bss_cla         : > RAMLS1
   cla_shared       : > RAMLS1
#if defined(__TI_EABI__)
   .const_cla       :   LOAD = FLASH4,
                        RUN = RAMLS1,
                        RUN_START(Cla1ConstRunStart),
                        LOAD_START(Cla1ConstLoadStart),
                        LOAD_SIZE(Cla1ConstLoadSize)
#else
   .const_cla       :   LOAD = FLASH2,
                        RUN = RAMLS1,
                        RUN_START(_Cla1ConstRunStart),
                        LOAD_START(_Cla1ConstLoadStart),
                        LOAD_SIZE(_Cla1ConstLoadSize)
#endif


   #if defined(__TI_EABI__)
       .TI.ramfunc : {} LOAD = FLASH4,
                        RUN = RAMD0,
                        LOAD_START(RamfuncsLoadStart),
                        LOAD_SIZE(RamfuncsLoadSize),
                        LOAD_END(RamfuncsLoadEnd),
                        RUN_START(RamfuncsRunStart),
                        RUN_SIZE(RamfuncsRunSize),
                        RUN_END(RamfuncsRunEnd),
                        ALIGN(8)
   #else
       .TI.ramfunc : {} LOAD = FLASH3,
                        RUN = RAMD0,
                        LOAD_START(_RamfuncsLoadStart),
                        LOAD_SIZE(_RamfuncsLoadSize),
                        LOAD_END(_RamfuncsLoadEnd),
                        RUN_START(_RamfuncsRunStart),
                        RUN_SIZE(_RamfuncsRunSize),
                        RUN_END(_RamfuncsRunEnd),
                        ALIGN(8)
   #endif

}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
