#include "I2C.h"
/* 
 * @function: void setupI2C(void)
 *
 * @param:    none
 * 
 *            sets up I2C outputs on iport_a(0x1100)
 */
void setupI2C(void)
{
	//I2C Port #2 - connected to PMOD Thermostat 
	//set to digital out
	PORTSetPinsDigitalOut(IOPORT_A, BIT_2 | BIT_3);

}

/* 
 * @function: startI2CTransfer(bool restart)
 *
 * @param:    bool restart     true:  starts a transmission
 * 							   false: waits for idle bus then sends new transmission
 * 
 * @return:                    true if successful, false otherwise
 *
 *				               a function that transmits over I2C
 */
bool startI2CTransfer(bool restart)
{
	I2C_STATUS Status;
	//send a start request
	if(restart) {
		I2CRepeatStart(PMODTmp_I2C_BUS);
	}
	else {
		//wait for Idle Bus, then start transfer
		while(!I2CBusIsIdle(PMODTmp_I2C_BUS));

		//throw error if collision
		if(I2CStart(PMODTmp_I2C_BUS) != I2C_SUCCESS) {
			DBPRINTF("ERROR: Bus collision during transfer Start\n");
			return false;
		}
	}
	//wait for signal to complete
	do {
		Status = I2CGetStatus(PMODTmp_I2C_BUS);
	}while(!(Status & I2C_START));

	return true;
}

/* 
 * @function: bool stopI2CTransfer(void)
 *
 * @param:    none
 * 
 * @return:                    true if successful stop, otherwise waits for stop
 *
 *				               a function that ends transmission over I2C
 */
bool stopI2CTransfer(void)
{
	I2C_STATUS status;
	//stop the transmission
	I2CStop(PMODTmp_I2C_BUS);
	//wait for signal to complete
	do {
		status = I2CGetStatus(PMODTmp_I2C_BUS);
	}while(!(status & I2C_STOP));

	return true;
}

/* 
 * @function: bool transmitOneByteI2C(UINT8 data)
 *
 * @param:    UINT8 data       a byte of data
 * 
 * @return:                    true if successful byte transmission, otherwise waits for stop
 *
 *				               returns true if no bus collision
 */
bool transmitOneByteI2C(UINT8 data)
{
	// Wait for the transmitter to be ready
	while(!I2CTransmitterIsReady(PMODTmp_I2C_BUS));

	// Transmit the byte 
	//if failue print out collision
	if(I2CSendByte(PMODTmp_I2C_BUS, data) == I2C_MASTER_BUS_COLLISION) {
		DBPRINTF("Error: I2C Master Bus Collision\n");
		return false;
	}
	// Wait for the transmission to finish
	while(!I2CTransmissionHasCompleted(PMODTmp_I2C_BUS));

	return true;
}