
/*******************************************************************************
 * Programmer: Bryan Clark and Kyle Vandelac                                                                *
 * Class: CptS 466                                                             *
 * Lab Project:                                                                *
 * Date:                                                                       *
 *                                                                             *
 * Description:  FreeRTOS Project Template                                     *
 *                                                                             *
 ******************************************************************************/

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


/* SYSCLK = 80 MHz (8 MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
   PBCLK = 40 MHz
   Primary Osc w/PLL (XT+,HS+,EC+PLL)
   WDT OFF
   Other options are don't care */
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

static void prvSetupHardware( void );

// Queue
xQueueHandle queueUART;
//Semaphores
static xSemaphoreHandle ocSema = NULL;

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

// Globals for setting up pmod CLS
const unsigned char enable_display[] = {27, '[', '3', 'e', '\0'};
const unsigned char set_cursor[] = {27, '[', '1', 'c', '\0'};
const unsigned char home_cursor[] = {27, '[', 'j', '\0'};
const unsigned char wrap_line[] = {27, '[', '0', 'h', '\0'};
const unsigned char clear[] = {27,'[', 'j', '\0'};
const unsigned char second_line[] = {27, '[', '1', ';', '0', 'H', '\0'};
const unsigned char back_light[] = {27, '[', '3', 'e', '\0'};
const unsigned char print_second[] = {27, '[', '1', '@', '\0'};

//wheel to get TMR value on
int left = 0;
int right = 0;
int pulse = 0;
int flagit = 0;

//INTERRUPTS
void __attribute__((interrupt(ipl5), vector (_EXTERNAL_3_VECTOR))) vEXT3InterruptWrapper (void);
void __attribute__((interrupt(ipl5), vector (_INPUT_CAPTURE_2_VECTOR))) vIC2InterruptWrapper (void);
void __attribute__((interrupt(ipl5), vector (_INPUT_CAPTURE_3_VECTOR))) vIC3InterruptWrapper (void);

int main (void)
{

    // Variable declarations
        unsigned int pb_clock = 0;
        pb_clock = SYSTEMConfigPerformance (SYSTEM_CLOCK);
    // Setup/initialize ports
    // Setup/initialize devices
        prvSetupHardware ();
        setupEXT3Int();
        //create semaphore
        vSemaphoreCreateBinary (ocSema);
        // Create Queue
        queueUART = xQueueCreate(2, sizeof(unsigned int) );
        if (queueUART != NULL)
         {
        xTaskCreate(vTaskWriteUART, "Write UART", 240,
                NULL, tskIDLE_PRIORITY + 3, NULL);
         xTaskCreate(vTaskGetTemp, "Get Temp", 240,
                NULL, tskIDLE_PRIORITY + 2, NULL);
         xTaskCreate(vTaskUpdateSpeed, "Update Speed", 240,
                NULL, tskIDLE_PRIORITY + 1, NULL);
        vTaskStartScheduler ();
         }

        // Should not reach this point!
    while (1) // Embedded programs run forever
    {
                char buffer[50];
        // Event loop
                sprintf(buffer, "Failed to Properly Execute!");
                //clear screen, output MPH
                putsUART2 (buffer);
    }

    return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////   TASKS   //////////////////////////////////////////////////
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
     //   BOOL              Acknowledged;
        BOOL              Success = TRUE;
        UINT8             i2cbyte;
    //set the I2C baudrate
    actualClock = I2CSetFrequency(PMODTmp_I2C_BUS, GetPeripheralClock(),I2C_CLOCK_FREQ);
    if(abs(actualClock-I2C_CLOCK_FREQ) > I2C_CLOCK_FREQ/10)
    {
        DBPRINTF("ERROR: I2C2 clcok frequenct (%u) error exceeds 10%%. \n", (unsigned)actualClock);

    }
while(1)
{
    //Enable the I2C bus
    I2CEnable(PMODTmp_I2C_BUS, TRUE);
    // Initialize data buffer
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, PMODTmp_ADDRESS, I2C_WRITE);
    i2cData[0] = SlaveAddress.byte;
  //  i2cData[1] = 0x4B; //location to read from(high address)
    DataSz = 1;
    //1. send start condition
    //Start the transfer
    //while less than 2?
    int getData = 0;
    //Data from I2C
    int TempData[2] = {};

while(getData < 2)
{
    if(!StartTransfer(FALSE))
    {
        while(1);
    }
    //2. send 7-bit slave address as write(the dummy write)
    //Transmit All data
    Index = 0;
    while(Success &&(Index < DataSz))
    {
        //transmit a byte
        if(TransmitOneByte(i2cData[Index]))
        {
            // Advance to the next Byte
            Index++;

            //verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(PMODTmp_I2C_BUS))
            {
                DBPRINTF("ERROR: Sent byte was not acknowledged.\n");
                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;
        }

    }
    //READ DATA BACK FROM PMODTmp
     //3. repeat start condition
    // Restart and send the PMODTmp's internal address to switch to a read transfer
    if(Success)
    {
        //3. repeat start condition
        // Send a Repeated Started condition
        if( !StartTransfer(TRUE) )
        {
            while(1);
        }

        //4. send seven bit slave address with read bit(R/W = 1)
        I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, PMODTmp_ADDRESS, I2C_READ);
        if (TransmitOneByte(SlaveAddress.byte))
        {
            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(PMODTmp_I2C_BUS))
            {
                DBPRINTF("Error: Sent byte was not acknowledged\n");
                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;
        }
    }

     //5. Read from buffer
    if(Success)
    {
        if(I2CReceiverEnable(PMODTmp_I2C_BUS, TRUE) == I2C_RECEIVE_OVERFLOW)
        {
            DBPRINTF("Error: I2C Receive Overflow\n");
            Success = FALSE;
        }
        else
        {
            while(!I2CReceivedDataIsAvailable(PMODTmp_I2C_BUS));
            i2cbyte = I2CGetByte(PMODTmp_I2C_BUS);
        }

    }

    // End the transfer (stop here if an error occured)
    StopTransfer();
    if(!Success)
    {
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
/*-----------------------------------------------------------*/
static void prvSetupHardware( void )
{
    /* Configure the hardware for maximum performance. */
    vHardwareConfigurePerformance();

    /* Setup to use the external interrupt controller. */
    vHardwareUseMultiVectoredInterrupts();

    portDISABLE_INTERRUPTS();

        //functions defined in SETUP section below
        setupHB();
        setup_Switches();
        setupSPI_ports();
        setupUART2();
        setup_SPI2();
        setupTMP();

        portENABLE_INTERRUPTS();

        mPORTESetPinsDigitalIn(0xF);
}
/*-----------------------------------------------------------*/
void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    taskDISABLE_INTERRUPTS();
    for( ;; );
}
/*-----------------------------------------------------------*/
void vApplicationIdleHook( void )
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time task stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook	function is
    called if a task stack overflow is detected.  Note the system/interrupt
    stack is not checked. */
    taskDISABLE_INTERRUPTS();
    for( ;; );
}
/*-----------------------------------------------------------*/
void vApplicationTickHook( void )
{
    /* This function will be called by each tick interrupt if
    configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
    added here, but the tick hook is called from an interrupt context, so
    code must not attempt to block, and only the interrupt safe FreeRTOS API
    functions can be used (those that end in FromISR()). */
}
/*-----------------------------------------------------------*/
void _general_exception_handler( unsigned long ulCause, unsigned long ulStatus )
{
    /* This overrides the definition provided by the kernel.  Other exceptions
    should be handled here. */
    for( ;; );
}
/*-----------------------------------------------------------*/
void vAssertCalled( const char * pcFile, unsigned long ulLine )
{
volatile unsigned long ul = 0;

    ( void ) pcFile;
    ( void ) ulLine;

    __asm volatile( "di" );
    {
        /* Set ul to a non-zero value using the debugger to step out of this
        function. */
        while( ul == 0 )
        {
            portNOP();
        }
    }
    __asm volatile( "ei" );
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////   Setups     //////////////////////////////////////////////////
////////////////////////     THIS SECTION DEFINES ALL THE STETUPS    ////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
//Sets up Switches for use
void setup_Switches(void)
{
    // Setup the two on-board buttons for read; input pins
    // According to the Digilent Cerebot MX4cK Reference Manual
    // BTN1 -> PA06, BTN2 -> PA07; NOTE: PA indicates PORTA

    // Prototype for necessary function as provided in <peripheral/ports.h>
    // void	PORTSetPinsDigitalIn(IoPortId portId, unsigned int inputs);
    // IOPORT_A and BIT_XX are defined in <peripheral/ports.h>
    PORTSetPinsDigitalIn (IOPORT_A, BIT_14 | BIT_15);
}
//Sets up Timers
void vSetupTimer2 (void)
{
    T2CON = 0;
    TMR2 = 0;
    PR2 = 1000;    // (unsigned short)( (configPERIPHERAL_CLOCK_HZ / (unsigned long)1000)-1);// usFrequencyHz
    T2CON = (BIT_15 | BIT_6 | BIT_3); //start the timer
}
//Sets up External Interrupts
void setupEXT3Int(void)
{
    ConfigINT3( EXT_INT_PRI_1 | RISING_EDGE_INT | EXT_INT_ENABLE );
}
void setupEXT4Int(void)
{
    ConfigINT4( EXT_INT_PRI_1 | RISING_EDGE_INT | EXT_INT_ENABLE );
}
//Sets up Oput Compare Modules
void OC3CONsetup(void)
{
    //MACROS
 //   PORTSetBits(IOPORT_D, (1<< 7) ); //set h bridge dir
    // The right most arguments of the OpenOC1 call represent the duty cycle of the output waveform
 //   OpenOC3( OC_ON | OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_IDLE_STOP | OC_PWM_FAULT_PIN_DISABLE, 0x5FFF, 0x5FFF );

        OC3CON = 0;
    OC3R = 0x5FFF;
    OC3RS = 0x5FFF;
    OC3CON = (
            /*			BIT_15 |	//oc1 on*/
            BIT_5 |		//16 bit mode
            BIT_2 |		//mode 110: pwm
            BIT_1 );	//	"

    /*enable the interrupt*/
    mOC3IntEnable( true );
    mOC3SetIntPriority( OC_PRI );
}
void OC2CONsetup(void)
{
    //MACROS
  //  PORTClearBits(IOPORT_D, (1 << 6) ); //set h bridge dir
    // The right most arguments of the OpenOC1 call represent the duty cycle of the output waveform
///    OpenOC2( OC_ON | OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_IDLE_STOP | OC_PWM_FAULT_PIN_DISABLE, 0x4FFF, 0x4FFF );
        OC2CON = 0;
    OC2R = 0x5FFF;
    OC2RS = 0x5FFF;
    OC2CON = (
            /*			BIT_15 |	//oc1 on*/
            BIT_5 |		//16 bit mode
            BIT_2 |		//mode 110: pwm
            BIT_1 );	//	"

    /*enable the interrupt*/
    mOC2IntEnable( true );
    mOC2SetIntPriority( OC_PRI );
}
//sets up motors for use
void setupHB ( void )
{
    //(1 <<8) for TrisD
    PORTSetPinsDigitalOut(IOPORT_D, (BIT_6) ); //Dir pin
    PORTSetPinsDigitalOut( IOPORT_D, (BIT_7) ); //Dir pin RIGHT WHEEL

    //(Enable pin for HBridge is bit 0 on port D)
    PORTSetPinsDigitalOut( IOPORT_D, BIT_1 ); //Enable pin
    PORTSetPinsDigitalOut( IOPORT_D, BIT_2 ); //Enable pin
    //Clear bits on port D
    PORTClearBits (IOPORT_D, BIT_1); // Make sure no waveform is outputted to Enable pin
     PORTClearBits (IOPORT_D, BIT_2); // Make sure no waveform is outputted to Enable pin
}
//sets up input capture from our HBridges
void setupInCap(void)
{
 //IC2
        IC2CON = 0;
    IC2CON = (
            BIT_15 |	//enable
            BIT_9 |		//rising edge
            BIT_7 |		//timer 2 source
            BIT_1 |		//011 - every rising edge
            BIT_0 );	//	"

    mIC2SetIntPriority( IC_PRI );
    mIC2SetIntSubPriority( 2 );
    mIC2IntEnable( true );
    mIC2ClearIntFlag();
  //IC3
        IC3CON = 0;
    IC3CON = (
            BIT_15 |	//enable
            BIT_9 |		//rising edge
            BIT_7 |		//timer 2 source
            BIT_1 |		//011 - every rising edge
            BIT_0 );	//	"

    mIC3SetIntPriority( IC_PRI );
    mIC3SetIntSubPriority( 1 );
    mIC3IntEnable( true );
    mIC3ClearIntFlag();
}
//A simple function that properly sets up SPI for PMOD Accelerometer
void setupSPI_ports(void)
{
    // Master Mode
        /* SDO2 - Output - RG08
       SDI2 - Input  - RG07
       SCK2 - Output - RG06
       SS2  - Output - RG09
       */

       PORTSetPinsDigitalOut(IOPORT_G, BIT_8 | BIT_6 | BIT_9);
       PORTSetPinsDigitalIn (IOPORT_G, BIT_7);
       PORTSetPinsDigitalOut(IOPORT_E, BIT_0);
}
// a companion function to the SPI ports
// sets up SPI on Channel 2
void setup_SPI2(void)
{
    SpiChnOpen(2, SPI_CON_MSTEN  | SPI_CON_MODE8 | SPI_CON_ON | CLK_POL_ACTIVE_LOW, 256);

    // Create a falling edge pin SS to start communication
    PORTSetBits (IOPORT_G, BIT_9);
    PORTClearBits (IOPORT_G, BIT_9);
}
//Sets up UART for use in displaying info to PMOD CLS screen
void setupUART2( void )
{
    //set up the ports
    PORTSetPinsDigitalIn( IOPORT_F, BIT_4 );
    PORTSetPinsDigitalOut( IOPORT_F, BIT_5 );

    OpenUART2(
            UART_EN |
            UART_IDLE_CON |
            UART_RX_TX |
            UART_DIS_WAKE |
            UART_DIS_LOOPBACK |
            UART_DIS_ABAUD |
            UART_NO_PAR_8BIT |
            UART_1STOPBIT |
            UART_IRDA_DIS |
            UART_MODE_FLOWCTRL |
            UART_DIS_BCLK_CTS_RTS |
            UART_NORMAL_RX |
            UART_BRGH_SIXTEEN,
            UART_TX_PIN_LOW |
            UART_RX_ENABLE |
            UART_TX_ENABLE |
            UART_INT_TX |
            UART_INT_RX_CHAR |
            UART_ADR_DETECT_DIS |
            UART_RX_OVERRUN_CLEAR,
            mUARTBRG( P_CLK, DESIRED_BAUD_RATE ) );

    return;
}
//sets up the port our PMODTMP is on
void setupTMP( void)
{
    //I2C Port #2 - connected to PMOD Thermostat 
	//set to digital out
    PORTSetPinsDigitalOut(IOPORT_A, BIT_2 | BIT_3);

}

/* 
*/
void vEXT3InterruptHandler (void)
{

    vSetupTimer2 ();
    //Turn on OCR
	//left wheel
    OC3CONsetup ();
	//right wheel
    OC2CONsetup (); 
    setupInCap();
    portBASE_TYPE taskWoken;
    xSemaphoreGiveFromISR (ocSema, &taskWoken);
    //Clear Flag
    IFS0bits.INT3IF = 0;
}
/* 
*/
void vIC2InterruptHandler (void)
{
    //store value of TMR2 
	//on IC1 Event into "left"
    left++;
    //Clear Flag
    IFS0bits.IC2IF = 0;
}
/* 
*/
void vIC3InterruptHandler (void)
{
     //update the number of pulses
	 pulse++;
     //store value of TMR2 
	 //on IC1 Event into "right" 
     right++;

    IFS0bits.IC3IF = 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////   I2C   ////////////////////////////////////////////////
////////////     THIS SECTION DEFINES ALL THE READING FOR PMOD TMP OVER I2C    //////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
//starts the transfer of data
BOOL StartTransfer(BOOL Restart)
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
BOOL StopTransfer(void)
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
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////  CALCULATIONS  ////////////////////////////////////////////
/////////////// THIS SECTION DEFINES ALL THE CALCULATIONS NEED TO DISPLAY TO PMOD  CLS  /////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
//converts I2C data into Celcius then Farenheit temp
double getFarenheit(int tempTemp)
{
//    These two bytes form a two?s
//complement 16-bit integer, if the result is
//shifted to the right three bits and multiplied by
//0.0625 the resulting signed floating point value
//will be a temperature reading in degrees
//Celsius
    double Celsius = 0.0;
    double Far;
    Celsius = tempTemp >> 3;
    Celsius *= .0625;
    Far = ((9/5)*Celsius) + 32;
    return Far;
}
//takes total number of pulses and converts them into MPH
double MilePerHour(int pulse, int prevPulse)
{
    //115 pulser per 7.85 inches
    double amp = 0.0;
    amp = .003878;
    double MPH = 0.0;
    int current_pulse = 0;

    current_pulse = pulse - prevPulse;

    MPH = amp * current_pulse;


   return MPH;
}
//Gets the total Distance Traveled based on number of pulses
double Distance(int pulse)
{
    //we know 7.85 inches for every 115 pulses therefore
    double distance = 0.0;
    distance = 7.85*(pulse/115);
    distance = distance/12;

   return distance;
}

/* 
 * @function: int forwardTen
 *
 * @param:    none
 * 
 *            counts the number of pulses 
 *            to go ten feet with the motors
 */
int forwardTen(void)
{
    int pulser = 0;
    int flag = 0;
    pulser = pulse;
    //go ten feet
    if(pulser == 1758) {
        flag = 1;
        OC2CON &= ~(1<< 15);
        OC3CON &= ~(1 << 15);
        delay(1000000);

    }

    return flag;
}

/*/* 
 * @function: void delay(unsigned int ms)
 *
 * @param:    ms             time in ms
 * 
 *            a simple no op delay
 */
void delay (unsigned int ms)
{
    int count = 0;
    for (count = 0; count < (ms * 40); count++) {
        asm ("nop");
    }
}
