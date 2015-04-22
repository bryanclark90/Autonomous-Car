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