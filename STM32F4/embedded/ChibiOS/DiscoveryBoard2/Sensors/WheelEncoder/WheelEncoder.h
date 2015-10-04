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

#ifndef WHEELENCODER_H
#define WHEELENCODER_H

#include "DiscoveryBoard.h"

typedef struct WheelEncoderDataT WheelEncoderDataT;

struct WheelEncoderDataT {
    int drivenDistanceLeftWheel;
    int drivenDistanceRightWheel;
    int speedLeftWheel;
    int speedRightWheel;
};

/**
 * This method initializes the wheel encoder ICU sensor.
 */
void initializeWheelEncoder(void);

/**
 * @return The associated thread for handling computing the vehicle speed.
 */
Thread* getThreadSpeed(void);

/**
 * This method transfers the wheel encoder ICU
 * data to a data structure the will be handled
 * as part of a protocol.
 *
 * @param Pointer to a data structure to return data.
 */
void getWheelEncoderData(WheelEncoderDataT *data);

/**
 * This method enables interactive access to the data
 * read from the wheel encoder ICU sensor and allows
 * the user to print the last read data.
 *
 * @chp Stream where to print the data to.
 * @argc Number of arguments for this command.
 * @argv List of arguments.
 */
void commandPrintWheelEncoder(BaseSequentialStream *chp, int argc, char *argv[]);

/**
 * This method resets the travelled paths.
 */
void commandResetWheelEncoder(BaseSequentialStream *chp, int argc, char *argv[]);

#endif // WHEELENCODER_H

