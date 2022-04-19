/*
 * Package.h
 *
 *  Created on: Jan 29, 2020
 *      Author: tdubuke
 */

#ifndef PACKAGE_H_
#define PACKAGE_H_

#include "Sensor.h"
#include "Actuator.h"
#include "PID.h"
#include "cmsis_os.h"
#include <string.h>

class Package {
public:
	Package(uint8_t nucleoPackage);

	void setPGain(float K_P);
	void setIGain(float K_I);
	void setDGain(float K_D);
	void setVelGain(float K_Vel);
	void setAccGain(float K_Acc);
	void setJerkGain(float K_Jerk);

	void initPID();

	void updateSensorValue();
	float getSensorValue();

	void StepPID();

	void addSensor(Sensor *sensor);
	void addActuator(Actuator *actuator);

	Sensor* getSensor();
	Actuator* getActuator();

	void newPosSetpoint(float pos);
	void newVelSetpoint(float vel);
	void newAccSetpoint(float acc);
	void torque(bool isOn);

	uint8_t getPackageNum();

	// Used to determine if the actuator handles pid control or we do
	bool needsPID = true;
private:
	uint8_t _nucleoPackage;

//	float _KP, _KI, _KD = 0;
	float _KVel, _KAcc, _KJer = 0;

	float _posSetpoint = NAN; //rad
	float _velSetpoint = NAN; //rad/sec
	float _accSetpoint = NAN; //rad/sec/sec

	bool newPos = false;

	float _posDelta = NAN;
	float _playtime = 0;

	float _sensorValue = 0; //rad

	Sensor *_sensor = 0;
	Actuator *_actuator = 0;

	PID *_PID = 0;
};

#endif /* PACKAGE_H_ */
