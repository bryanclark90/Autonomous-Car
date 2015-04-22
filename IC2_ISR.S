#include <p32xxxx.h>
#include <sys/asm.h>
#include "Source/portable/MPLAB/PIC32MX/ISR_Support.h"
	.set	nomips16
	.set 	noreorder
 	
	.extern vIC2InterruptHandler
	.extern xISRStackTop
	.global	vIC2InterruptWrapper

	.set	noreorder
	.set 	noat
	.ent	vIC2InterruptWrapper

vIC2InterruptWrapper:

	portSAVE_CONTEXT
	jal vIC2InterruptHandler
	nop
	portRESTORE_CONTEXT

	.end	vIC2InterruptWrapper

