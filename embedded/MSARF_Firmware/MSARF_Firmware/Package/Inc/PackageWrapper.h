/*
 * PackageWrapper.h
 *
 *  Created on: Jan 29, 2020
 *      Author: tdubuke
 */

#ifndef PACKAGEWRAPPER_H_
#define PACKAGEWRAPPER_H_

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Package Package;
void Package_StepPID(Package *obj);
void Package_VelSetpoint(Package *obj, int32_t vel);
void Package_PosSetpoint(Package *obj, int32_t pos);

uint8_t Package_getPackageNum(Package *obj);

void Package_updateSensorValue(Package *obj);

#ifdef __cplusplus
}
#endif

#endif /* PACKAGEWRAPPER_H_ */
