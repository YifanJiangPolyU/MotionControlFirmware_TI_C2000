
MEMORY
{
PAGE 0 :  /* Program Memory */
          /* Memory (RAM/FLASH) blocks can be moved to PAGE1 for data allocation */
          /* BEGIN is used for the "boot to Flash" bootloader mode   */

   BEGIN           	: origin = 0x080000, length = 0x000002
   RAMM0           	: origin = 0x000122, length = 0x0002DE
   RAMD0           	: origin = 0x00B000, length = 0x000800
   RAMM1           : origin = 0x000400, length = 0x000400
   RAMD1           : origin = 0x00B800, length = 0x000800
   RAMLS0          	: origin = 0x008000, length = 0x000800
   RAMLS1          	: origin = 0x008800, length = 0x000800
   RAMLS2      		: origin = 0x009000, length = 0x000800
   RAMLS3      		: origin = 0x009800, length = 0x000800
   RAMLS4      		: origin = 0x00A000, length = 0x000800
   RAMLS5      : origin = 0x00A800, length = 0x000800

   RAMGS0      : origin = 0x00C000, length = 0x001000
   RAMGS1      : origin = 0x00D000, length = 0x001000
   RAMGS2      : origin = 0x00E000, length = 0x001000
   RAMGS3      : origin = 0x00F000, length = 0x001000
   RAMGS4      : origin = 0x010000, length = 0x001000
   RAMGS5      : origin = 0x011000, length = 0x001000
   RAMGS6      : origin = 0x012000, length = 0x001000
   RAMGS7      : origin = 0x013000, length = 0x001000
   RAMGS8      : origin = 0x014000, length = 0x001000
   RAMGS9      : origin = 0x015000, length = 0x001000
   RAMGS10     : origin = 0x016000, length = 0x001000
   RAMGS11     : origin = 0x017000, length = 0x001000
   RAMGS12     : origin = 0x018000, length = 0x001000
   RAMGS13     : origin = 0x019000, length = 0x001000
   RAMGS14          : origin = 0x01A000, length = 0x001000
   RAMGS15          : origin = 0x01B000, length = 0x001000

   BOOT_RSVD       : origin = 0x000002, length = 0x000120
   RESET           	: origin = 0x3FFFC0, length = 0x000002

   CPU2TOCPU1RAM   : origin = 0x03F800, length = 0x000400
   CPU1TOCPU2RAM   : origin = 0x03FC00, length = 0x000400

   CLA1_MSGRAMLOW   : origin = 0x001480, length = 0x000080
   CLA1_MSGRAMHIGH  : origin = 0x001500, length = 0x000080

   FLASHD           : origin = 0x086000, length = 0x002000	/* on-chip Flash */

PAGE 1 :
         /* Flash sectors */
         FLASHA           : origin = 0x080002, length = 0x001FFE	/* on-chip Flash */
         FLASHB           : origin = 0x082000, length = 0x002000	/* on-chip Flash */
         FLASHC           : origin = 0x084000, length = 0x002000	/* on-chip Flash */
         FLASHE           : origin = 0x088000, length = 0x008000	/* on-chip Flash */
         FLASHF           : origin = 0x090000, length = 0x008000	/* on-chip Flash */
         FLASHG           : origin = 0x098000, length = 0x008000	/* on-chip Flash */
         FLASHH           : origin = 0x0A0000, length = 0x008000	/* on-chip Flash */
         FLASHI           : origin = 0x0A8000, length = 0x008000	/* on-chip Flash */
         FLASHJ           : origin = 0x0B0000, length = 0x008000	/* on-chip Flash */
         FLASHK           : origin = 0x0B8000, length = 0x002000	/* on-chip Flash */
         FLASHL           : origin = 0x0BA000, length = 0x002000	/* on-chip Flash */
         FLASHM           : origin = 0x0BC000, length = 0x002000	/* on-chip Flash */
         FLASHN           : origin = 0x0BE000, length = 0x002000	/* on-chip Flash */

}

SECTIONS
{
   /* Allocate program areas: */
   codestart           : > BEGIN       PAGE = 0, ALIGN(4)
   .cinit              : > FLASHB      PAGE = 1, ALIGN(4)
   .pinit              : > FLASHB,     PAGE = 1, ALIGN(4)
   .text               : >> FLASHB | FLASHC | FLASHE      PAGE = 1, ALIGN(4)
   .reset              : > RESET,     PAGE = 0, TYPE = DSECT /* not used, */

   /* Allocate uninitalized data sections: */
   .stack              : > RAMD0        PAGE = 0
   .ebss               : >> RAMGS0 | RAMGS1 | RAMGS2       PAGE = 0
   .esysmem            : > RAMGS3 PAGE = 0
   .cio                : > RAMGS3 PAGE = 0
   .binit              :  >RAMGS3 PAGE = 0

   /* Initalized sections go in Flash */
   .econst             : >> FLASHE | FLASHF | FLASHG      PAGE = 1, ALIGN(4)
   .switch             : > FLASHB      PAGE = 1, ALIGN(4)

   /* CLA program */
   Cla1Prog         : LOAD = FLASHD,
                      RUN = RAMLS0,
                      LOAD_START(_Cla1funcsLoadStart),
                      LOAD_END(_Cla1funcsLoadEnd),
                      RUN_START(_Cla1funcsRunStart),
                      LOAD_SIZE(_Cla1funcsLoadSize),
                      PAGE = 0, ALIGN(4)
   CLADataLS1		: > RAMLS1, PAGE=0
   Cla1ToCpuMsgRAM  : > CLA1_MSGRAMLOW,   PAGE = 0
   CpuToCla1MsgRAM  : > CLA1_MSGRAMHIGH,  PAGE = 0

   .const_cla       : LOAD = FLASHD,
                      RUN = RAMLS1,
                      LOAD_START(_Cla1ConstLoadStart),
                      LOAD_END(_Cla1ConstLoadEnd),
                      RUN_START(_Cla1ConstRunStart),
                      LOAD_SIZE(_Cla1ConstLoadSize),
                      PAGE = 0, ALIGN(4)

   .scratchpad      : > RAMLS1,       PAGE = 0

   CPU1DataRAM      : > RAMGS3,       PAGE = 0

#ifdef __TI_COMPILER_VERSION__
	#if __TI_COMPILER_VERSION__ >= 15009000
	.TI.ramfunc : {} LOAD = FLASHD,
						 RUN = RAMM0 | RAMGS3,
                         LOAD_START(_RamfuncsLoadStart),
                         LOAD_SIZE(_RamfuncsLoadSize),
                         LOAD_END(_RamfuncsLoadEnd),
                         RUN_START(_RamfuncsRunStart),
                         RUN_SIZE(_RamfuncsRunSize),
                         RUN_END(_RamfuncsRunEnd),
						 PAGE = 0, ALIGN(4)
	#else
   ramfuncs            : LOAD = FLASHD,
                         RUN = RAMM0 | RAMGS3,
                         LOAD_START(_RamfuncsLoadStart),
                         LOAD_SIZE(_RamfuncsLoadSize),
                         LOAD_END(_RamfuncsLoadEnd),
                         RUN_START(_RamfuncsRunStart),
                         RUN_SIZE(_RamfuncsRunSize),
                         RUN_END(_RamfuncsRunEnd),
                         PAGE = 0, ALIGN(4)
    #endif
#endif

   /* The following section definitions are required when using the IPC API Drivers */
    GROUP : > CPU1TOCPU2RAM, PAGE = 0
    {
        PUTBUFFER
        PUTWRITEIDX
        GETREADIDX
    }

    GROUP : > CPU2TOCPU1RAM, PAGE = 0
    {
        GETBUFFER :    TYPE = DSECT
        GETWRITEIDX :  TYPE = DSECT
        PUTREADIDX :   TYPE = DSECT
    }
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
