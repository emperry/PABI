/*
 * PID.h
 *
 *  Created on: Feb 1, 2020
 *      Author: tdubuke
 */

#ifndef PID_H_
#define PID_H_

class PID {
public:
	PID( float dt, float max, float min, float Kp, float Kd, float Ki );
	~PID();

	void setPGain(float K_P);
	void setIGain(float K_I);
	void setDGain(float K_D);

	float calculate( float setpoint, float pv );
private:
	float _dt;
	float _max;
	float _min;
	float _Kp;
	float _Kd;
	float _Ki;
	float _pre_error;
	float _integral;
};

#endif /* PID_H_ */
