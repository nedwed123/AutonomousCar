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

#ifndef LEDS_H
#define LEDS_H

enum LED { ORANGE = 1, 
           GREEN,
           RED,
           BLUE };

enum LED_STATE { OFF = 0,
                 ON };

/**
 * This method enables the STM32F4 Discovery Board LEDs.
 */
void enableLEDs(void);

/**
 * This method changes the status of the specified LED.
 *
 * @led LED that should be changes.
 * @state State of the LED.
 */
void changeStateLED(enum LED led, enum LED_STATE state);

/**
 * This method turns off the STM32F4 Discovery Board LEDs.
 */
void turnOffLEDs(void);

#endif // USER_SHELL_H

