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

#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#define USE_ONBOARD_ACCELEROMETER  1 // If this is set to 1, the onboard accelerometer can be read. Please be aware that using this sensor limits the RC receiver to read only to two input channels because TIM14 at pin PA7 is in conflict with this sensor.
#define USE_ONBOARD_TEMPERATURE    0 // If the onboard temperature sensor is used the infrared sensors cannot be used!
#define USE_INFRARED               1 // Support reading values from analog infrared sensors (not available if the temperature sensor is used!).
#define USE_ULTRASONIC             1 // Support reading values from I2C ultrasonic sensors.
#define USE_RCRECEIVER             1 // Support for reading ICU data from an RC-receiver hand set. If the onboard accelerometer is not used, up to three channels are supported (otherwise, only two channels are supported).
#define USE_WHEELENCODER           1 // Support for reading ICU data from a four channel wheel encoder. 
#define USE_RAZORBOARD             1 // Support for reading serial data from a 9DoF razor board.
#define USE_ACTORS                 1 // Support for interfacing with steering and acceleration actors.

// The following switches enable the various data feeds from STM32F4 Discovery Board to the host.
#define ONBOARD_ACCELEROMETER_FEED 1
#define ONBOARD_TEMPERATURE_FEED   2
#define INFRARED_FEED              4
#define ULTRASONIC_FEED            8
#define WHEELENCODER_FEED          16
#define RAZORBOARD_FEED            32

#endif // PERIPHERALS_H

