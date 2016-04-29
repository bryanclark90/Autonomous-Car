/*******************************************************************************
 * Programmer:                                                                 *
 * Class: CptS 466                                                             *
 * Lab Project:                                                                *
 * Date:                                                                       *
 *                                                                             *
 * Description:  FreeRTOS Project Template                                     *
 *                                                                             *
 ******************************************************************************/

// #include all necessary standard and user-defined libraries
#include <plib.h> // Includes all major functions and macros required to develop
                  // programs for the PIC32MX4
#include <p32xxxx.h> // Need specific PIC32 names for memory regions

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "ConfigPerformance.h"

// Place your #pragma statements here, or in another .h file;
// #pragma statements are used to set your operating clock frequency

/* SYSCLK = 80 MHz (8 MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 40 MHz
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care */

/* Oscillator Settings
*/
#pragma config FNOSC = PRIPLL // Oscillator selection
#pragma config POSCMOD = EC // Primary oscillator mode
#pragma config FPLLIDIV = DIV_2 // PLL input divider
#pragma config FPLLMUL = MUL_20 // PLL multiplier
#pragma config FPLLODIV = DIV_1 // PLL output divider
#pragma config FPBDIV = DIV_2 // Peripheral bus clock divider
#pragma config FSOSCEN = OFF // Secondary oscillator enable

// Place your #define constants and macros here, or in another .h file

// Yes, don't forget your prototypes
// Prototypes go here, or in a .h file, which you would also need to #include
static void prvSetupHardware( void );

// Tasks
void vTaskOnLEDs (void *pvParameters);
void vTaskOffLEDs (void *pvParameters);

int main (void)
{
	// Variable declarations

	// Setup/initialize ports
	// Setup/initialize devices
        prvSetupHardware ();

        // Can you draw the execution pattern diagram for these tasks?
        xTaskCreate (vTaskOnLEDs, "LEDs On", configMINIMAL_STACK_SIZE, NULL,
                     tskIDLE_PRIORITY + 1, NULL);
        xTaskCreate (vTaskOffLEDs, "LEDs Off", configMINIMAL_STACK_SIZE, NULL,
                     tskIDLE_PRIORITY + 1, NULL);

        vTaskStartScheduler ();

        // Should not reach this point!
	while (1) // Embedded programs run forever
	{
		// Event loop
	}

	return 0;
}

// This task turn on the LEDs
void vTaskOnLEDs (void *pvParameters)
{
    while (1)
    {
        LATBSET = 0x3C00; // Turn LEDs on

        // Causes task to go to Blocked state until period has expired
        vTaskDelay (1000 / portTICK_RATE_MS); // 1 s delay
    }
}

// This task turns off the LEDS
void vTaskOffLEDs (void *pvParameters)
{
    while (1)
    {
        LATBCLR = 0x3C00; // Turn LEDs off

        // Causes task to go to Blocked state until period has expired
        vTaskDelay (1500 / portTICK_RATE_MS); // 1.5 s delay
    }
}

/*************************************************************
 * Function:                                                 *
 * Date Created:                                             *
 * Date Last Modified:                                       *
 * Description:                                              *
 * Input parameters:                                         *
 * Returns:                                                  *
 * Usages:                                                   *
 * Preconditions:                                            *
 * Postconditions:                                           *
 *************************************************************/

// Put other function definitions below main (), or they
// may go in another .c source file; Functions most likely
// will include port and device setups/initalizations;
// Be sure to comment all functions with the above block

/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Configure the hardware for maximum performance. */
	vHardwareConfigurePerformance();

	/* Setup to use the external interrupt controller. */
	vHardwareUseMultiVectoredInterrupts();

	portDISABLE_INTERRUPTS();

        TRISBCLR = 0x3C00; // Set on-board LEDs as outputs
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time task stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook	function is
	called if a task stack overflow is detected.  Note the system/interrupt
	stack is not checked. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
}
/*-----------------------------------------------------------*/

void _general_exception_handler( unsigned long ulCause, unsigned long ulStatus )
{
	/* This overrides the definition provided by the kernel.  Other exceptions
	should be handled here. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vAssertCalled( const char * pcFile, unsigned long ulLine )
{
volatile unsigned long ul = 0;

	( void ) pcFile;
	( void ) ulLine;

	__asm volatile( "di" );
	{
		/* Set ul to a non-zero value using the debugger to step out of this
		function. */
		while( ul == 0 )
		{
			portNOP();
		}
	}
	__asm volatile( "ei" );
}