
//gets Temperature from PMODTMP using i2c
void vTaskGetTemp(void *pvParameters)
{
	int tempTemp = 0;
	xQueueReceive(queueUART, &tempTemp, 100);
	//////////////////////// I2C //////////////////////////////////////
	//initialize the data buffer
	UINT8             i2cData[3] ={};
	I2C_7_BIT_ADDRESS SlaveAddress;
	int               Index;
	int               DataSz;
	UINT32            actualClock;
	bool              Success = true;
	UINT8             i2cbyte;
	//set the I2C baudrate
	actualClock = I2CSetFrequency(PMODTmp_I2C_BUS, GetPeripheralClock(),I2C_CLOCK_FREQ);
	if(abs(actualClock-I2C_CLOCK_FREQ) > I2C_CLOCK_FREQ/10){
		DBPRINTF("ERROR: I2C2 clcok frequenct (%u) error exceeds 10%%. \n", (unsigned)actualClock);

	}
	while(1) {
		//Enable the I2C bus
		I2CEnable(PMODTmp_I2C_BUS, true);
		// Initialize data buffer
		I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, PMODTmp_ADDRESS, I2C_WRITE);
		i2cData[0] = SlaveAddress.byte;
  		DataSz = 1;
		//1. send start condition
		//Start the transfer
		//while less than 2?
		int getData = 0;
		//Data from I2C
		int TempData[2] = {};

		while(getData < 2) {
			//start a transfer
			if(!startI2CTransfer(false)) {
				while(1);
			}
			//2. send 7-bit slave address as write(the dummy write)
			//Transmit All data
			Index = 0;
			while(Success && (Index < DataSz)) {
				//transmit a byte
				if(transmitOneByteI2C(i2cData[Index])) {
					// Advance to the next Byte
					Index++;

					//verify that the byte was acknowledged
					if(!I2CByteWasAcknowledged(PMODTmp_I2C_BUS)) {
						DBPRINTF("ERROR: Sent byte was not acknowledged.\n");
						Success = false;
					}
				}
				else {
					Success = false;
				}

			}
			//READ DATA BACK FROM PMODTmp
		 	//3. repeat start condition
			// Restart and send the PMODTmp's internal address to switch to a read transfer
			if(Success) {
				//3. repeat start condition
				// Send a Repeated Started condition
				if(!startI2CTransfer(false) ){
					while(1);
				}

				//4. send seven bit slave address with read bit(R/W = 1)
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

		 	//5. Read from buffer
			if(Success) {
				if(I2CReceiverEnable(PMODTmp_I2C_BUS, true) == I2C_RECEIVE_OVERFLOW) {
					DBPRINTF("Error: I2C Receive Overflow\n");
					Success = false;
				}
				else {
					while(!I2CReceivedDataIsAvailable(PMODTmp_I2C_BUS));
					i2cbyte = I2CGetByte(PMODTmp_I2C_BUS);
				}

			}

			// End the transfer (stop here if an error occured)
			stopI2CTransfer();
			if(!Success) {
				while(1);
			}
			//index the data
			TempData[getData] = i2cbyte;
			getData++;

		}

		int quickTemp = TempData[0];
		double Far;
		quickTemp = quickTemp << 8;
		quickTemp += TempData[1];
		tempTemp = quickTemp;
		///////////////////////////////////////////////
		//PLACE THIS ONTO THE QUEUE!
		xQueueSend(queueUART, &tempTemp, 100);

		vTaskDelay(.1 / portTICK_RATE_MS);
	}
}

void vTaskWriteUART (void *pvParameters)
{
	int pulse = 0;
	int tempTemp = 0;
	int prevPulse;
	char bufferMPHtime[50];
	char bufferDisttemp[50];
	//kill me
	int time = 0;
	double currentDist = 0.0;
	double currentMPH = 0.0;
	double currentTemp;
	double avgTemp;
	int FiveCount = 0;

	while(1)
	{
		 xQueueReceive(queueUART, &tempTemp, 100);
		//calculate distance MPH and temp
		 currentDist = Distance(pulse);
		currentMPH = MilePerHour(pulse, prevPulse);
		currentTemp = getFarenheit(tempTemp);
		//print to buffer
		sprintf(bufferMPHtime, "%.3f MPH %d Sec", currentMPH, time);
		//update time and previous pulse count
		time++;
		prevPulse = pulse;
		putsUART2(clear);
		putsUART2(bufferMPHtime);
		putsUART2(second_line);
		//SpiChnPutS(2,second_line,6);
		if(FiveCount <= 10)
		{
			sprintf(bufferDisttemp, "%.2f ft %.1f F", currentDist, currentTemp);
			avgTemp += currentTemp;
			if(FiveCount == 10)
			{
				avgTemp = (avgTemp/5);
			}
		}
		else if(FiveCount <= 12)
		{
			sprintf(bufferDisttemp, "%.2f ft %.1f AF", currentDist, avgTemp);
		}
		else if(FiveCount > 12)
		{
			sprintf(bufferDisttemp, "%.2f ft %.1f AF", currentDist, avgTemp);
			avgTemp = 0;
			//  iterations = 0;
			FiveCount = 0;
		}
		FiveCount++;
		putsUART2(bufferDisttemp);
	   xQueueSend(queueUART, &tempTemp, 100);
	   vTaskDelay(5 / portTICK_RATE_MS); // 1 s delay
	}
}

//update speed for OC motors
void vTaskUpdateSpeed (void *pvParameters)
{
	while(1)
	{
		xSemaphoreTake (ocSema, portMAX_DELAY);

		 //difference between right and elft
		   double diff = 0;
			 if(left >= right)
			  {
				 diff = (right/left);
				 OC2RS = 0x6FFF *(1+(1-diff));
				 OC3RS = 0x6FFF * (diff/3);
			   }
			 else if(left < right)
			   {
				 diff = (left/right);
				 OC3RS = 0x6FFF *(1+(1-diff));
				 OC2RS = 0x6FFF * (diff/3);
			   }
		   forwardTen();
		}
		   vTaskDelay(.1 / portTICK_RATE_MS);
  }