/*
 * ChibiOS/RT Application Layer for Miniature Cars
 * Copyright (C) 2013 - 2015 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef STEERING_ACCELERATION_H
#define STEERING_ACCELERATION_H

#include "DiscoveryBoard.h"

/**
 * This method initializes the actors.
 */
void initializeSteeringAcceleration(void);

/**
 * @return The associated thread for handling the acceleration motor.
 */
Thread* getThreadAcceleration(void);

/**
 * @return The associated thread for handling the steering servo.
 */
Thread* getThreadSteering(void);

/**
 * This method sets the desired steering and
 * speed values.
 *
 * @param steering Desired steering wheel angle.
 * @param speed Desired speed.
 */
void setMotorData(int steering, int speed);

/**
 * This method enables interactive access to the control
 * the steering servo and acceleration motor and allows
 * the user to print the last read data.
 *
 * @chp Stream where to print the data to.
 * @argc Number of arguments for this command.
 * @argv List of arguments.
 */
void commandControlSteeringAccelerationMotors(BaseSequentialStream *chp, int argc, char *argv[]);

#endif // STEERING_ACCELERATION_H

