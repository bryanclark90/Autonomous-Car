/*-----------------------------------------------------------*/
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
