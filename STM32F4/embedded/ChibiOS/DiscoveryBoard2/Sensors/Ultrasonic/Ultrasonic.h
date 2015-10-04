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

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "DiscoveryBoard.h"

typedef struct UltrasonicDataT UltrasonicDataT;

struct UltrasonicDataT {
    int address;
    int distance;

    UltrasonicDataT *next;
};

/**
 * This method initializes the ultrasonic sensors.
 */
void initializeUltrasonic(void);

/**
 * @return The associated thread for handling the ultrasonic sensors.
 */
Thread* getThreadUltrasonic(void);

/**
 * This method transfers the distances measured
 * by the ultrasonic sensors to a data structure that
 * will be handled as part of a protocol.
 *
 * @param Pointer to a data structure to return data.
 */
void getUltrasonicData(UltrasonicDataT *data);

/**
 * This method enables interactive access to the data
 * read from the ultrasonic sensor and allows the user
 * to print the last read data.
 *
 * @chp Stream where to print the data to.
 * @argc Number of arguments for this command.
 * @argv List of arguments.
 */
void commandPrintUltrasonicDistances(BaseSequentialStream *chp, int argc, char *argv[]);

/**
 * This method enables interactive access to change
 * the I2C bus address of the ultrasonic sensor.
 *
 * @chp Stream where to print the data to.
 * @argc Number of arguments for this command.
 * @argv List of arguments.
 */
void commandSetUltrasonicAddress(BaseSequentialStream *chp, int argc, char *argv[]);

/**
 * This method enables interactive access to trigger
 * one measurement of all ultrasonic sensors.
 *
 * @chp Stream where to print the data to.
 * @argc Number of arguments for this command.
 * @argv List of arguments.
 */
void commandTriggerSingleUltrasonicMeasurement(BaseSequentialStream *chp, int argc, char *argv[]);

/**
 * This method enables interactive access to start
 * continuous measurements from all ultrasonic sensors.
 *
 * @chp Stream where to print the data to.
 * @argc Number of arguments for this command.
 * @argv List of arguments.
 */
void commandStartContinuousUltrasonicMeasurements(BaseSequentialStream *chp, int argc, char *argv[]);

/**
 * This method enables interactive access to stop
 * continuous measurements from all ultrasonic sensors.
 *
 * @chp Stream where to print the data to.
 * @argc Number of arguments for this command.
 * @argv List of arguments.
 */
void commandStopContinuousUltrasonicMeasurements(BaseSequentialStream *chp, int argc, char *argv[]);

#endif // ULTRASONIC_H

