/* 
 * @function: double calcFarenheit(int unfilteredTemp)
 *
 * @param:    int unfilteredTemp  These two bytes form a two?s
 *			                      complement 16-bit integer,
 *
 * @returns:  temperature in Farenhiet 
 *
 *            if the temp is shifted to the right three bits 
 *            and multiplied by 0.0625 the resulting signed 
 *            floating point value will be a temperature reading 
 *            in degrees Celsius. Using this can convert to farenhiet by 
 *            multiplying that by  9/5 and adding 32
 */
double calcFarenheit(int unfilteredTemp)
{
	double celsius = unfilteredTemp >> 3;
	celcius *=  *.0625;
	return ((9/5)*(celsius) + 32);
}

/* 
 * @function: double calcMPH(int prevPulse)
 *
 * @param:    none
 *
 * @returns:  the average MPH since last function call
 *
 *            multiplies the difference in pulses since last call by 7.85 
 *            in(diamete of wheel)/115 pulses(per one full diameter)
 *            and returns this value in MPH
 */
double calcMPH(void)
{
	static int lastPulse = 0;
	//115 pulses per 7.85 inches
	static const double amp = .003878;
	//number of pulses between last calculation
	int pulseWidth = gPulse - lastPulse;
	lastPulse = gPulse;

	return (double)(amp * pulseWidth); 
}

/* 
 * @function: double calcDistance(int pulse)
 *
 * @param:    none
 *
 * @returns:  distance traveled in feet
 *
 *            multiplies number of pules by 7.85 in(diamete of wheel)/115 pulses(per one full diameter)
 *            and divides by 12 inches in a foot to get distance in feet
 */
double calcDistance(void)
{
	//we know 7.85 inches for every 115 pulses therefore
	return (7.85*(gPulse/115))/12;
}

/* 
 * @function: int forwardTen
 *
 * @param:    none though the global pulse is used to check against
 *
 * @returns:  true if have gone ten feet, false otherwise
 *
 *            counts the number of pulses 
 *            to go ten feet with the motors
 */
bool calcIfTen(void)
{
	//go ten feet and then disbale motors
	if(gPulse >= 1758) {
		OC2CON &= ~(1<< 15);
		OC3CON &= ~(1 << 15);
		delay(1000000);
		return true;

	}
	return false;
}