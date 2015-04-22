#include "main.h"

#define I2C_CLOCK_FREQ           100000

//I2C
void setupI2C(void);
BOOL TransmitOneByte(UINT8 data);
BOOL stopI2CTransfer(void);
BOOL startI2CTransfer(BOOL Restart);