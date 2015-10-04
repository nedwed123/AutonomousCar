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

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

uint32_t yaw(int razorData[12]);
uint32_t magnetometer(int razorData[12]);
uint32_t gyroscope(int razorData[12]);
uint32_t accelerometer_razor(int razorData[12]);
uint32_t accelerometer_discovery(int8_t discoveryData[2]);
uint8_t ultrasonic(int value);
uint32_t infrared(int ir1, int ir2, int ir3);
uint32_t currentPos(float imuInfo[7]);
uint32_t traveled_path(float imuInfo[7]);
uint32_t current_velocity(float imuInfo[7]);
uint32_t current_orientation(float imuInfo[7]);
void translate(int request, int ir_data[3], int16_t us_data[3],int razorData[12], float imuInfo[7], int8_t discoveryAccelData[2], uint8_t data[4]);

#endif // PROTOCOL_H

