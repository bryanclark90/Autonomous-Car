#include <p32xxxx.h>
#include <sys/asm.h>
#include "Source/portable/MPLAB/PIC32MX/ISR_Support.h"
	.set	nomips16
	.set 	noreorder
 	
	.extern vIC3InterruptHandler
	.extern xISRStackTop
	.global	vIC3InterruptWrapper

	.set	noreorder
	.set 	noat
	.ent	vIC3InterruptWrapper

vIC3InterruptWrapper:

	portSAVE_CONTEXT
	jal vIC3InterruptHandler
	nop
	portRESTORE_CONTEXT

	.end	vIC3InterruptWrapper

