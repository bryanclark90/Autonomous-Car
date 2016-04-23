
/*******************************************************************************
 * Programmer: Bryan Clark                                                     *
 * Email: bryan.clark@email.wsu.edu                                            *
 * Phone: 509-294-4161                                                         *
 ******************************************************************************/
#include "main.h"
#include "tasks.h"
#include "calculations.h"
#include "setupPeripherals.h"


int main (void)
{
	unsigned int pb_clock = 0;
	pb_clock = SYSTEMConfigPerformance (SYSTEM_CLOCK);
	// setup/initialize ports and devices
	prvSetupHardware ();
	//create semaphore
	vSemaphoreCreateBinary (ocSema);
	// Create Queue
	queueUART = xQueueCreate(2, sizeof(unsigned int) );
	if (queueUART != NULL) {
		xTaskCreate(vTaskWriteUART, "Write UART", 240,
				NULL, tskIDLE_PRIORITY + 3, NULL);
		 xTaskCreate(vTaskGetTemp, "Get Temp", 240,
				NULL, tskIDLE_PRIORITY + 2, NULL);
		 xTaskCreate(vTaskUpdateSpeed, "Update Speed", 240,
				NULL, tskIDLE_PRIORITY + 1, NULL);
		vTaskStartScheduler ();
	}

	// Should not reach this point!
	while (1) {
		char buffer[50];
		// Event loop
		sprintf(buffer, "Failed to Properly Execute!");
		//clear screen, output MPH
		putsUART2 (buffer);
	}

	return 0;
}

