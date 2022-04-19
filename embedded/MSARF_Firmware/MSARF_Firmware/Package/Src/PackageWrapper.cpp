/*
 * PackageWrapper.cpp
 *
 *  Created on: Jan 29, 2020
 *      Author: tdubuke
 */

#include "PackageWrapper.h"
#include "Package.h"

extern "C" {
	void Package_StepPID(Package *obj){
		obj->StepPID();
	}

	void Package_updateSensorValue(Package *obj){
		obj->updateSensorValue();
	}

	void Package_VelSetpoint(Package *obj, int32_t vel){
		obj->newVelSetpoint(vel);
	}

	void Package_PosSetpoint(Package *obj, int32_t pos){
		obj->newPosSetpoint(pos);
	}

	uint8_t Package_getPackageNum(Package *obj){
		return obj->getPackageNum();
	}
}

