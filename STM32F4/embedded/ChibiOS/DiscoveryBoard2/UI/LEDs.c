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

#include "DiscoveryBoard.h"

void enableLEDs(void) {
    palSetPadMode(GPIOD, GPIOD_LED3, PAL_MODE_OUTPUT_PUSHPULL); // Orange
    palSetPadMode(GPIOD, GPIOD_LED4, PAL_MODE_OUTPUT_PUSHPULL); // Green
    palSetPadMode(GPIOD, GPIOD_LED5, PAL_MODE_OUTPUT_PUSHPULL); // Red
    palSetPadMode(GPIOD, GPIOD_LED6, PAL_MODE_OUTPUT_PUSHPULL); // Blue
}

void changeStateLED(enum LED led, enum LED_STATE state) {
    switch (led) {
        case ORANGE:
            if (state == ON)
                palSetPad(GPIOD, GPIOD_LED3);
            else
                palClearPad(GPIOD, GPIOD_LED3);
        break;
        case GREEN:
            if (state == ON)
                palSetPad(GPIOD, GPIOD_LED4);
            else
                palClearPad(GPIOD, GPIOD_LED4);
        break;
        case RED:
            if (state == ON)
                palSetPad(GPIOD, GPIOD_LED5);
            else
                palClearPad(GPIOD, GPIOD_LED5);
        break;
        case BLUE:
            if (state == ON)
                palSetPad(GPIOD, GPIOD_LED6);
            else
                palClearPad(GPIOD, GPIOD_LED6);
        break;
    }
}

void turnOffLEDs(void) {
    changeStateLED(ORANGE, OFF);
    changeStateLED(GREEN, OFF);
    changeStateLED(RED, OFF);
    changeStateLED(BLUE, OFF);
}

