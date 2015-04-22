//sets up the port our PMODTMP is on
void setupI2C(void)
{
	//I2C Port #2 - connected to PMOD Thermostat 
	//set to digital out
	PORTSetPinsDigitalOut(IOPORT_A, BIT_2 | BIT_3);

}




/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////   I2C   ////////////////////////////////////////////////
////////////     THIS SECTION DEFINES ALL THE READING FOR PMOD TMP OVER I2C    //////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
//starts the transfer of data
BOOL startI2CTransfer(BOOL Restart)
{
	I2C_STATUS Status;

	//send a start request
	if(Restart) {
		I2CRepeatStart(PMODTmp_I2C_BUS);

	}
	else {
		//Wait for Idle Bus, then start transfer
		while(!I2CBusIsIdle(PMODTmp_I2C_BUS));

		//throw error
		if(I2CStart(PMODTmp_I2C_BUS) != I2C_SUCCESS) {
			DBPRINTF("ERROR: Bus collision during transfer Start\n");
			return FALSE;
		}
	}
	//wait for signal to complete
	do {
		Status = I2CGetStatus(PMODTmp_I2C_BUS);
	}while(!(Status & I2C_START));

	return TRUE;
}
//stops the trasnfer of data
BOOL stopI2CTransfer(void)
{
	I2C_STATUS Status;

	//stop the transmission
	I2CStop(PMODTmp_I2C_BUS);
	//wait for signal to complete
	do {
		Status = I2CGetStatus(PMODTmp_I2C_BUS);
	}while(!(Status & I2C_STOP));

	return TRUE;
}

//transmits one byte of data
BOOL TransmitOneByte(UINT8 data)
{
	// Wait for the transmitter to be ready
	while(!I2CTransmitterIsReady(PMODTmp_I2C_BUS));

	// Transmit the byte
	if(I2CSendByte(PMODTmp_I2C_BUS, data) == I2C_MASTER_BUS_COLLISION) {
		DBPRINTF("Error: I2C Master Bus Collision\n");
		return FALSE;
	}

	// Wait for the transmission to finish
	while(!I2CTransmissionHasCompleted(PMODTmp_I2C_BUS));

	return TRUE;
}