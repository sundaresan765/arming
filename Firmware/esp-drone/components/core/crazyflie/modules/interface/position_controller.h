/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#ifndef POSITION_CONTROLLER_H_
#define POSITION_CONTROLLER_H_

#include "stabilizer_types.h"

//dks
#include "sensors_mpu6050_hm5883L_ms5611.h"

float asl;
extern float targetAltitude;
float currentAltitude;
extern bool altHoldMode;
extern bool less_voltage;
extern bool armMode;
extern bool isThrust;
extern bool negative_thrust;
extern bool disarm_clicked;
extern bool isTakeOff;
extern bool landMode;
extern bool isArmsuccess;
extern bool disarm;
extern bool landed;
extern bool isthrust;
//extern bool takeOffMode;
extern  bool isTake_thrust;
extern bool takeoff_completed;
extern bool land_completed;
extern int32_t rawThrust;
extern float MAX_ALTITUDE;
extern float voltage;
extern bool  isTake_button;
extern bool ThrustPos;
extern bool boostTakeoffvelocity; // using for takeoff
extern bool boosThrvelocity; // using for takeoff manual thrust
extern float takeOffHeight;

float computeAltitudeHoldPID(float currentAltitude);
// float computeAltitudesHoldPID(float currentAltitude);

// A position controller calculate the thrust, roll, pitch to approach
// a 3D position setpoint
void positionControllerInit();
void positionControllerResetAllPID();
void positionController(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state);
void velocityController(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state);

bool wifiSendData(uint32_t size, uint8_t *data);
void disarmMotor();

#endif /* POSITION_CONTROLLER_H_ */
