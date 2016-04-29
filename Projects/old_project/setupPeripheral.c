#include "setupPeripheral.h"

/* 
 * @function: void prvSetupHardware( void )
 *
 * @param:    none
 * 
 * @return:   peripherals set up
 *
 *			  calls all the setups 
 *							  
 */
static void prvSetupHardware( void )
{
	//configure the hardware for maximum performance.
	vHardwareConfigurePerformance();
	//setup to use the external interrupt controller.
	vHardwareUseMultiVectoredInterrupts();
	//disable all interrupts while setting up hardware
	portDISABLE_INTERRUPTS();
	
	//set up peropherals
	setupHB();
	setup_Switches();
	setupSPI_ports();
	setupUART2();
	setup_SPI2();
	setupI2C();
	//reenable interrupts
	portENABLE_INTERRUPTS();

	mPORTESetPinsDigitalIn(0xF);

	//set up external interrupt on motors
	setupEXT3Int();
}

/* 
 * @function: void setup_Switches(void)
 *
 * @param:    none
 * 
 * @return:   peripherals set up
 *
 *			  sets up the switches for the cerebot 
 *							  
 */
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

/* 
 * @function: void vSetupTimer2 (void)
 *
 * @param:    none
 * 
 * @return:   turns on timer 2
 *							  
 */
void vSetupTimer2 (void)
{
	T2CON = 0;
	TMR2 = 0;
	PR2 = 1000;    // (unsigned short)( (configPERIPHERAL_CLOCK_HZ / (unsigned long)1000)-1);// usFrequencyHz
	T2CON = (BIT_15 | BIT_6 | BIT_3); //start the timer
}

/* 
 * @function: void setupEXT3Int(void)
 *
 * @param:    none
 * 
 * @return:   sets up external interrupt 4
 *							  
 */
void setupEXT3Int(void)
{
	ConfigINT3( EXT_INT_PRI_1 | RISING_EDGE_INT | EXT_INT_ENABLE );
}

/* 
 * @function: void OC3CONsetup(void)
 *
 * @param:    none
 * 
 * @return:   sets up OC3
 *							  
 */
void OC3CONsetup(void)
{
	OC3CON = 0;
	OC3R = 0x5FFF;
	OC3RS = 0x5FFF;
	OC3CON = (
			BIT_5 |		//16 bit mode
			BIT_2 |		//mode 110: pwm
			BIT_1 );	//	"

	/*enable the interrupt*/
	mOC3IntEnable( true );
	mOC3SetIntPriority( OC_PRI );
}

/* 
 * @function: void OC2CONsetup(void)
 *
 * @param:    none
 * 
 * @return:   sets up OC2
 *							  
 */
void OC2CONsetup(void)
{

	OC2CON = 0;
	OC2R = 0x5FFF;
	OC2RS = 0x5FFF;
	OC2CON = (
			BIT_5 |		//16 bit mode
			BIT_2 |		//mode 110: pwm
			BIT_1 );	//	"

	/*enable the interrupt*/
	mOC2IntEnable( true );
	mOC2SetIntPriority( OC_PRI );
}

/* 
 * @function: void setupHB ( void )
 *
 * @param:    none
 * 
 * @return:   sets up the HBridge which drives the OC control of 2 and 3
 *							  
 */
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

/* 
 * @function: void setupHB ( void )
 *
 * @param:    none
 * 
 * @return:   sets up input capture on the HBridge=
 *							  
 */
void setupInCap(void)
{
 //IC2
	IC2CON = 0;
	IC2CON = (
			BIT_15 |	//enable
			BIT_9 |		//rising edge
			BIT_7 |		//timer 2 source
			BIT_1 |		//011 - every rising edge
			BIT_0 );

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
			BIT_0 );

	mIC3SetIntPriority( IC_PRI );
	mIC3SetIntSubPriority( 1 );
	mIC3IntEnable( true );
	mIC3ClearIntFlag();
}

/* 
 * @function: void setupSPI_ports(void)
 *
 * @param:    none
 * 
 * @return:   simple function that properly sets up SPI for PMOD Accelerometer
 *							  
 */
void setupSPI_ports(void)
{
	PORTSetPinsDigitalOut(IOPORT_G, BIT_8 | BIT_6 | BIT_9);
	PORTSetPinsDigitalIn (IOPORT_G, BIT_7);
	PORTSetPinsDigitalOut(IOPORT_E, BIT_0);
}

/* 
 * @function: void setupSPI_ports(void)
 *
 * @param:    none
 * 
 * @return:   sets up SPI on Channel 2
 *	
 *            a companion function to the SPI ports						  
 */
void setup_SPI2(void)
{
	SpiChnOpen(2, SPI_CON_MSTEN  | SPI_CON_MODE8 | SPI_CON_ON | CLK_POL_ACTIVE_LOW, 256);

	// Create a falling edge pin SS to start communication
	PORTSetBits (IOPORT_G, BIT_9);
	PORTClearBits (IOPORT_G, BIT_9);
}

/* 
 * @function: void setupSPI_ports(void)
 *
 * @param:    none
 * 
 * @return:   sets up UART for use in displaying info to PMOD CLS screen
 *							  
 */
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


/* 
 * @function: void vEXT3InterruptHandler (void)
 *
 * @param:    none
 * 
 * @return:   when interrupt 3 fires, it sets up timer 2
 *            and starts the wheels. This interrupt is tied to
 *            to the switch
 *							  
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
 * @function: void vIC2InterruptHandler (void)
 *
 * @param:    none
 * 
 * @return:   fires when ever the motor turns one pulse
 *            updates the global gLeft, to keep track of number
 *            of times the left motor has turned
 *							  
 */
void vIC2InterruptHandler (void)
{
	//store value of TMR2 
	//on IC1 Event into "left"
	gLeft++;
	//Clear Flag
	IFS0bits.IC2IF = 0;
}

/* 
 * @function: void vIC3InterruptHandler (void)
 *
 * @param:    none
 * 
 * @return:   fires when ever the motor turns one pulse
 *            updates the global gReft, to keep track of number
 *            of times the right motor has turned, and updates
 *            gPulse to use for distance measurements.
 *							  
 */
 void vIC3InterruptHandler (void)
{
	 //update the number of pulses
	 gPulse++;
	 //store value of TMR2 
	 //on IC1 Event into "right" 
	 gRight++;

	IFS0bits.IC3IF = 0;
}
