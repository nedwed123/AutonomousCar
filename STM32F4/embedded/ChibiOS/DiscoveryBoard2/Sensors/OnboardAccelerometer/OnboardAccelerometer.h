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

#ifndef ONBOARD_ACCELEROMETER_H
#define ONBOARD_ACCELEROMETER_H

#include "DiscoveryBoard.h"

typedef struct OnboardAccelerometerDataT OnboardAccelerometerDataT;

struct OnboardAccelerometerDataT {
    int8_t x;
    int8_t y;
    int8_t z;
};

/**
 * This method initializes the onboard accelerometer.
 */
void initializeOnboardAccelerometer(void);

/**
 * @return The associated thread for handling the onboard accelerometer.
 */
Thread* getThreadOnboardAccelerometer(void);

/**
 * This method transfers the onboard accelerometer
 * data to a data structure the will be handled
 * as part of a protocol.
 *
 * @param Pointer to a data structure to return data.
 */
void getOnboardAccelerometerData(OnboardAccelerometerDataT *data);

/**
 * This method enables interactive access to the data
 * read from the onboard accelerometer and allows the
 * user to print the last read data.
 *
 * @chp Stream where to print the data to.
 * @argc Number of arguments for this command.
 * @argv List of arguments.
 */
void commandPrintAccelerometer(BaseSequentialStream *chp, int argc, char *argv[]);

#endif // ONBOARD_ACCELEROMETER_H

