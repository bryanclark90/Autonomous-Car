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

#define PMODTmp_I2C_BUS    I2C2
#define PMODTmp_ADDRESS    0x4B 

/*interrupt priorities*/
#define OC_PRI 3			//output compare priority
#define IC_PRI 3			//input capture priority
#define TIMER_PRI 2			//lcd priority
#define SWITCH_PRI 1		//switch isr
#define P_CLK 40000000		//peripheral clock override

//interrupts
void __attribute__((interrupt(ipl5), vector (_EXTERNAL_3_VECTOR))) vEXT3InterruptWrapper (void);
void __attribute__((interrupt(ipl5), vector (_INPUT_CAPTURE_2_VECTOR))) vIC2InterruptWrapper (void);
void __attribute__((interrupt(ipl5), vector (_INPUT_CAPTURE_3_VECTOR))) vIC3InterruptWrapper (void);

//create the bool type
typedef enum { false, true } bool;

//create a queue handler
xQueueHandle queueUART;
//create a semaphore
static xSemaphoreHandle ocSema = NULL;

// Globals for setting up pmod CLS
const unsigned char enable_display[] = {27, '[', '3', 'e', '\0'};
const unsigned char set_cursor[] = {27, '[', '1', 'c', '\0'};
const unsigned char home_cursor[] = {27, '[', 'j', '\0'};
const unsigned char wrap_line[] = {27, '[', '0', 'h', '\0'};
const unsigned char clear[] = {27,'[', 'j', '\0'};
const unsigned char second_line[] = {27, '[', '1', ';', '0', 'H', '\0'};
const unsigned char back_light[] = {27, '[', '3', 'e', '\0'};
const unsigned char print_second[] = {27, '[', '1', '@', '\0'};

//Globals wheel to get TMR value on
volatile int gLeft = 0;
volatile int gRight = 0;
volatile int gPulse = 0;

//no op(found in utility.h)
void delay (unsigned int ms);