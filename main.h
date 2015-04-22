#include <plib.h>
#include <p32xxxx.h>
#include <peripheral/system.h>		//uart
#include <peripheral/timer.h>		//timers
#include <peripheral/uart.h>		//uart
#include <peripheral/ports.h>		//port definitions
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "ConfigPerformance.h"
#include "semphr.h"

/* Oscillator Settings*/
#pragma config FNOSC = PRIPLL // Oscillator selection
#pragma config POSCMOD = EC // Primary oscillator mode
#pragma config FPLLIDIV = DIV_2 // PLL input divider
#pragma config FPLLMUL = MUL_20 // PLL multiplier
#pragma config FPLLODIV = DIV_1 // PLL output divider
#pragma config FPBDIV = DIV_2 // Peripheral bus clock divider
#pragma config FSOSCEN = OFF // Secondary oscillator enable

#define SYSTEM_CLOCK		80000000
#define DESIRED_BAUD_RATE	9600

#define GetSystemClock()         (SYSTEM_CLOCK)
#define GetPeripheralClock()     (SYSTEM_CLOCK)
#define GetInstructionClock()    (SYSTEM_CLOCK)
#define I2C_CLOCK_FREQ           100000

#define PMODTmp_I2C_BUS    I2C2
#define PMODTmp_ADDRESS    0x4B 

/*interrupt priorities*/
#define OC_PRI 3			//output compare priority
#define IC_PRI 3			//input capture priority
#define TIMER_PRI 2			//lcd priority
#define SWITCH_PRI 1		//switch isr
#define P_CLK 40000000		//peripheral clock override
#define true 1
#define false 0

static void prvSetupHardware( void );
// Tasks
void vTaskWriteUART (void *pvParameters);
void vTaskUpdateSpeed (void *pvParameters);
void vTaskGetTemp(void *pvParameters);
// SPI
void setupSPI_ports (void);
void setup_SPI2 (void);
//UART
void setupUART2( void );
// Buttons
void setup_Switches(void);
//I2C
void setupTMP(void);
BOOL TransmitOneByte(UINT8 data);
BOOL StopTransfer(void);
BOOL StartTransfer(BOOL Restart);
//Motors
void setupOC(void);
void setupHB (void);
//Calc Functions
double getFarenheit(int tempTemp);
double MilePerHour(int pulse, int prevPulse);
double Distance(int pulse);
//Speed and Forward
void checkSpeed(void);
int forwardTen(void);
//no op
void delay (unsigned int ms);
//interrupts
void setupEXT3Int(void);