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

#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include "DiscoveryBoard.h"

typedef struct TemperatureDataT TemperatureDataT;

struct TemperatureDataT {
    int8_t T;
};

/**
 * This method initializes the onboard temperature sensor.
 */
void initializeTemperature(void);

/**
 * @return The associated thread for handling the temperature sensor.
 */
Thread* getThreadTemperature(void);

/**
 * This method transfers the onboard temperature
 * data to a data structure the will be handled
 * as part of a protocol.
 *
 * @param Pointer to a data structure to return data.
 */
void getTemperatureData(TemperatureDataT *data);

/**
 * This method enables interactive access to the data
 * read from the onboard temperature sensor and allows
 * the user to print the last read data.
 *
 * @chp Stream where to print the data to.
 * @argc Number of arguments for this command.
 * @argv List of arguments.
 */
void commandPrintTemperature(BaseSequentialStream *chp, int argc, char *argv[]);

#endif // TEMPERATURE_H

