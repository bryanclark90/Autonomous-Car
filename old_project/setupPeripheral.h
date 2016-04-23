#include "main.h"

static void prvSetupHardware( void );

// SPI
void setupSPI_ports (void);
void setup_SPI2 (void);
//UART
void setupUART2( void );
// Buttons
void setup_Switches(void);

//Motors
void setupOC(void);
void setupHB (void);

//interrupts
void setupEXT3Int(void);