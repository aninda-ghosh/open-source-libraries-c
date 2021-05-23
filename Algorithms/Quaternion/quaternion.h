/*
 * quaternion.h
 *
 *  Created on: May 3, 2021
 *      Author: aninda
 */

#ifndef INC_QUATERNION_H_
#define INC_QUATERNION_H_

#include "math.h"

#define invSampleFreq   (1.0f/500.0f)   	// sample frequency in Hz
#define beta         	10.0f            	// 2 * proportional gain 0.1 - 0.5 - 5

class QUATERNION {
private:
	float q0;
	float q1;
	float q2;
	float q3;   // quaternion of sensor frame relative to auxiliary frame
	float invSqrt(float x);
public:
	QUATERNION();
	void updateMARG(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	void computeAnglesRad(float *dest);
	void computeAnglesDeg(float *dest);
};

#endif /* INC_QUATERNION_H_ */
