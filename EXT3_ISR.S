#include <p32xxxx.h>
#include <sys/asm.h>
#include "Source/portable/MPLAB/PIC32MX/ISR_Support.h"

	.set	nomips16
	.set 	noreorder
 	
	.extern vEXT3InterruptHandler
	.extern xISRStackTop
	.global	vEXT3InterruptWrapper

	.set	noreorder
	.set 	noat
	.ent	vEXT3InterruptWrapper

vEXT3InterruptWrapper:

	portSAVE_CONTEXT
	jal vEXT3InterruptHandler
	nop
	portRESTORE_CONTEXT

	.end	vEXT3InterruptWrapper

