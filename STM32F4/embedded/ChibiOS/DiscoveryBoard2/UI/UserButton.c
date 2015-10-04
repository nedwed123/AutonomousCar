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

int isUserButtonPressed(unsigned int timeOut) {
    const int FIVE_HUNDRED_MILLISECONDS = 500;

    // User button is connected to PA0.
    palSetPadMode(GPIOA, 0, PAL_MODE_INPUT);

    int userButtonPressed = 0;
    while (timeOut > 0) {
        timeOut--;

        if (!userButtonPressed) {
            userButtonPressed = palReadPad(GPIOA, 0);
        }

        // Indicate the checking process by blinking the LEDs
        if (userButtonPressed) {
            changeStateLED(RED, ON);
            chThdSleepMilliseconds(FIVE_HUNDRED_MILLISECONDS);
            changeStateLED(RED, OFF);
            chThdSleepMilliseconds(FIVE_HUNDRED_MILLISECONDS);
        }
        else {
            changeStateLED(BLUE, ON);
            chThdSleepMilliseconds(FIVE_HUNDRED_MILLISECONDS);
            changeStateLED(BLUE, OFF);
            chThdSleepMilliseconds(FIVE_HUNDRED_MILLISECONDS);
        }
    }

    return userButtonPressed;
}

