/*
 * iirfilter.cpp
 *
 *  Created on: May 3, 2021
 *      Author: aninda
 */
#include "iirfilter.h"

IIRFILTER::IIRFILTER(float a, float b, float c, float d, float e, float f, float cutoff_freq) {
	float T = 2 * atan(cutoff_freq * PI / 2);
	float CommonDenominator = ((4 * a) + (C * T * T) + (2 * b * T));
	b2 = ((-2 * e * T) + (4 * d) + (f * T * T)) / CommonDenominator;
	b1 = ((2 * f * T * T) - (8 * d)) / CommonDenominator;
	b0 = ((4 * d) + (f * T * T) + (2 * e * T)) / CommonDenominator;
	a2 = ((-2 * b * T) + (4 * a) + (c * T * T)) / CommonDenominator;
	a1 = ((2 * c * T * T) - (8 * a)) / CommonDenominator;
	a0 = 1;
}

float IIRFILTER::filter(float value) {
	float y;
	y = (b0 * w[0]) + (b1 * w[1]) + (b2 * w[2]);
	w[0] = value - (a1 * w[1]) - (a2 * w[2]);
	w[1] = w[0];
	w[2] = w[1];
	return y;
}

