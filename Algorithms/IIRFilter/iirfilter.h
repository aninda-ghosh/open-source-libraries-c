/*
 * iirfilter.h
 *
 *  Created on: May 3, 2021
 *      Author: anind
 */

#ifndef INC_IIRFILTER_H_
#define INC_IIRFILTER_H_

#include <math.h>

/*
 * Implementation for the Filter
 * https://dspguru.com/dsp/howtos/implement-iir-filters/
 * https://en.wikipedia.org/wiki/Digital_biquad_filter
 * https://en.wikipedia.org/wiki/Digital_biquad_filter#Implementation
 *
 * Taken from the 6 Pole butterworth filter design
 * http://www.iowahills.com/A4IIRBilinearTransform.html
 * http://www.iowahills.com/Example%20Code/IIRSecondOrderImplementation.txt
 */
#define D 				(0)
#define E				(0)
#define F				(1)
#define A				(1)
#define B				(1.931851)
#define C				(1)
#define CUTOFF_FREQ		(0.2)
#define PI      		(3.14159265)

class IIRFILTER {
private:
	float a0, a1, a2;
	float b0, b1, b2;
	float w[3];
public:
    IIRFILTER(float a=A, float b=B, float c=C, float d=D, float e=E, float f=F, float cutoff_freq = CUTOFF_FREQ);

    float filter(float value);
};



#endif /* INC_IIRFILTER_H_ */
