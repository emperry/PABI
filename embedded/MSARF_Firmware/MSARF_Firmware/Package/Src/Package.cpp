/*
 * Package.cpp
 *
 *  Created on: Jan 29, 2020
 *      Author: tdubuke
 */

#include "Package.h"

Package::Package(uint8_t nucleoPackage): _nucleoPackage(nucleoPackage) {}

void Package::StepPID(){
	if(_actuator == 0){
		return;
	}

	if(!isnan(_posSetpoint)){
		// Do position control off-board without repeated commands
		if(_sensor != nullptr && _PID == nullptr && newPos){
			_actuator->move(_posSetpoint, _playtime);
			newPos = false;
		}

		// On-board PID control
		else if(_sensor != nullptr && _PID != nullptr){
			float pid_percent = _PID->calculate(_posSetpoint, _sensorValue);
			_actuator->move(_posSetpoint, pid_percent);
		}

		// Open loop control
		else if(_sensor == nullptr && _PID == nullptr && !isnan(_velSetpoint)){
			_actuator->move(_posSetpoint, _velSetpoint);
		}
	}
}

void Package::torque(bool isOn){
	if(_actuator != 0){
		_actuator->torque(isOn);
	}
}

void Package::initPID(){
	PID *pid_ptr = (PID *)pvPortMalloc(sizeof(PID));
	PID pid(0.02, 100, -100, 0, 0, 0);

	memcpy(pid_ptr, &pid, sizeof(PID));

	_PID = pid_ptr;
}

float Package::getSensorValue(){
	return _sensorValue;
}

void Package::updateSensorValue(){
	if(_sensor != nullptr){
		float value = _sensor->getValue();
		if(value != -1){
			_sensorValue = value;
		}
	}
}

void Package::newPosSetpoint(float pos){
	_posSetpoint = pos;

	if(!isnan(_sensorValue)){
		_posDelta = _sensorValue - _posSetpoint;
	}

	newPos = true;
}
void Package::newVelSetpoint(float vel){
	_velSetpoint = vel;

	if(!isnan(_posDelta)){
		_playtime = abs(_posDelta) / _velSetpoint;
	}
}
void Package::newAccSetpoint(float acc){
	_accSetpoint = acc;
}

uint8_t Package::getPackageNum(){
	return _nucleoPackage;
}

void Package::setPGain(float K_P){
	if(_PID != 0)
		_PID->setPGain(K_P);
}
void Package::setIGain(float K_I){
	if(_PID != 0)
		_PID->setIGain(K_I);
}
void Package::setDGain(float K_D){
	if(_PID != 0)
		_PID->setDGain(K_D);
}
void Package::setVelGain(float K_Vel){
	_KVel = K_Vel;
}
void Package::setAccGain(float K_Acc){
	_KAcc = K_Acc;
}
void Package::setJerkGain(float K_Jerk){
	_KJer = K_Jerk;
}

void Package::addSensor(Sensor *sensor){
	_sensor = sensor;
}
void Package::addActuator(Actuator *actuator){
	_actuator = actuator;
}
Sensor* Package::getSensor(){
	return _sensor;
}
Actuator* Package::getActuator(){
	return _actuator;
}
