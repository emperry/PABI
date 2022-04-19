/*
 * PID.cpp
 *
 *  Created on: Feb 1, 2020
 *      Author: tdubuke
 */

#include "PID.h"
#include <cmath>

PID::PID( float dt, float max, float min, float Kp, float Kd, float Ki ):
	_dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{}

PID::~PID(){}

float PID::calculate( float setpoint, float pv )
{
	// Calculate error
	float error = setpoint - pv;

	// Proportional term
	float Pout = _Kp * error;

	// Integral term
	_integral += error * _dt;
	float Iout = _Ki * _integral;

	// Derivative term
	float derivative = (error - _pre_error) / _dt;
	float Dout = _Kd * derivative;

	// Calculate total output
	float output = Pout + Iout + Dout;

	// Restrict to max/min
	if( output > _max )
		output = _max;
	else if( output < _min )
		output = _min;

	// Save error to previous error
	_pre_error = error;

	return output;
}

void PID::setPGain(float K_P){
	_Kp = K_P;
}
void PID::setIGain(float K_I){
	_Ki = K_I;
}
void PID::setDGain(float K_D){
	_Kd = K_D;
}
