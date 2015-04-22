#include "main.h"

#define I2C_CLOCK_FREQ           100000

//I2C
void setupI2C(void);
bool startI2CTransfer(bool restart);
bool stopI2CTransfer(void);
bool transmitOneByteI2C(UINT8 data);