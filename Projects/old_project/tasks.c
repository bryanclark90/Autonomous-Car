
#include "tasks.h"
/* 
 * @function: void vTaskGetTemp(void *pvParameters)
 *
 * @param:    void *pvParameters
 * 
 * @return:   nothing
 *
 *			  pulls data from the I2C to get the temperature
 *							  
 */
void vTaskGetTemp(void *pvParameters)
{
	int queueTemp = 0;
	xQueueReceive(queueUART, &queueTemp, 100);
	
	//initialize the data buffer
	UINT8 i2cData[3] ={};
	UINT32 actualClock;
	//set the I2C baudrate
	actualClock = I2CSetFrequency(PMODTmp_I2C_BUS, GetPeripheralClock(),I2C_CLOCK_FREQ);
	//throw error if bad frequency
	if(abs(actualClock-I2C_CLOCK_FREQ) > I2C_CLOCK_FREQ/10){
		DBPRINTF("ERROR: I2C2 clock frequency (%u) error exceeds 10%%. \n", (unsigned)actualClock);

	}
	while(1) {
		//Enable the I2C bus
		I2CEnable(PMODTmp_I2C_BUS, true);
		// Initialize data buffer
		I2C_7_BIT_ADDRESS SlaveAddress;
		I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, PMODTmp_ADDRESS, I2C_WRITE);
		i2cData[0] = SlaveAddress.byte;
		//1. send start condition
		//Start the transfer
		int byte = 0;
		//Data from I2C
		int dataRecieved[2] = {};
		bool success = true;
		//retrieve 2 bytes
		while(byte < 2) {
			//start a transfer
			if(!startI2CTransfer(false)) {
				while(1);
			}
			//send 7-bit slave address as write(the dummy write)
			//transmit All data
			int index = 0;
			while(success && (index < 1)) {
				//transmit a byte
				if(transmitOneByteI2C(i2cData[index])) {
					// Advance to the next Byte
					index++;
					//verify that the byte was acknowledged
					if(!I2CByteWasAcknowledged(PMODTmp_I2C_BUS)) {
						DBPRINTF("ERROR: Sent byte was not acknowledged.\n");
						success = false;
					}
				}
				else {
					success = false;
				}

			}
			//read data back from pmodtemp
			//restart and send the PMODTmp's internal address to switch to a read transfer
			if(success) {
				// Send a Repeated Started condition
				if(!startI2CTransfer(false) ){
					while(1);
				}

				//send seven bit slave address with read bit(R/W = 1)
				I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, PMODTmp_ADDRESS, I2C_READ);
				if (transmitOneByteI2C(SlaveAddress.byte)) {
					// Verify that the byte was acknowledged
					if(!I2CByteWasAcknowledged(PMODTmp_I2C_BUS)) {
						DBPRINTF("Error: Sent byte was not acknowledged\n");
						Success = false;
					}
				}
				else {
					Success = false;
				}
			}
		 	//read from buffer
			if(success) {
				//verify wasn't overflow, if was throw error
				if(I2CReceiverEnable(PMODTmp_I2C_BUS, true) == I2C_RECEIVE_OVERFLOW) {
					DBPRINTF("Error: I2C Receive Overflow\n");
					Success = false;
				}
				else {
					while(!I2CReceivedDataIsAvailable(PMODTmp_I2C_BUS));
					UINT8 i2cbyte; 
					i2cbyte = I2CGetByte(PMODTmp_I2C_BUS);
				}

			}
			// End the transfer (stop here if an error occured)
			stopI2CTransfer();
			if(!Success) {
				while(1);
			}
			//index the data
			dataRecieved[byte] = i2cbyte;
			//move to next back
			byte++;
		}
		//place the data in the queue
		queueTemp = (dataRecieved[0] << 8) + dataRecieved[1];
		xQueueSend(queueUART, &tempTemp, 100);

		vTaskDelay(.1 / portTICK_RATE_MS);
	}
}

/* 
 * @function: void vTaskWriteUART (void *pvParameters) (void *pvParameters)
 *
 * @param:    void *pvParameters
 * 
 * @return:   nothing
 *
 *			  calculates the latest speed, distance, time and temperature
 *            then displays that to the screen and then sleeps for one sec before
 *		      waking up and repeating
 *							  
 */
void vTaskWriteUART (void *pvParameters)
{
	int queueTemp = 0;
	int time = 0;
	//loop through task
	while(1) {
		 xQueueReceive(queueUART, &queueTemp, 100);
		//update time
		time++;
		//clear the display and put first two lines
		putsUART2(clear);
		double currentMPH = calcMPH();
		//print to buffer
		char bufferMPHtime[50];
		sprintf(bufferMPHtime, "%.3f MPH %d Sec", currentMPH, time);
		putsUART2(bufferMPHtime);
		//move cursor to second line using macros
		putsUART2(second_line);
		//calculate distance MPH and temp
		double currentDist = calcDistance();
		double currentTemp = calcFarenheit(queueTemp);
		char bufferTemp[50];
		sprintf(bufferTemp, "%.2f ft %.1f F", currentDist, currentTemp);
		//place distance traveled and temperature on the display
		putsUART2(bufferTemp);
	    xQueueSend(queueUART, &queueTemp, 100);
	    vTaskDelay(5 / portTICK_RATE_MS); // 1 s delay 
	}
}

/* 
 * @function: void vTaskUpdateSpeed (void *pvParameters)
 *
 * @param:    void *pvParameters
 * 
 * @return:   nothing
 *
 *			  updates the speed with which the motors are turning
 *			  used to keep the car driving straight by adjusting motors speed
 *            up and down if it is moving faster or slower than the opposite motor
 *							  
 */
void vTaskUpdateSpeed (void *pvParameters)
{
	while(!calcIfTen()) {
		xSemaphoreTake (ocSema, portMAX_DELAY);
		//difference between right and elft
		double diff = 0.0;
		if(gLeft >= gRight) {
			diff = (gRight/gLeft);
			OC2RS = 0x6FFF *(1+(1-diff));
			OC3RS = 0x6FFF * (diff/3);
		}
		else if(gLeft < gRight) {
			diff = (gLeft/gRight);
			OC3RS = 0x6FFF *(1+(1-diff));
			OC2RS = 0x6FFF * (diff/3);
		}
	}
    vTaskDelay(.1 / portTICK_RATE_MS);
}
