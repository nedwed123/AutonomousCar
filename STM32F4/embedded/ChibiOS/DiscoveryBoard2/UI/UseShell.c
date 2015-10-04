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

static int HAS_SHELL = 0;

int hasShell(void) {
    return HAS_SHELL;
}

int useShell(void) {
    enableLEDs();
    turnOffLEDs();

    const int USER_BUTTON_TIMEOUT = 5;
    HAS_SHELL = isUserButtonPressed(USER_BUTTON_TIMEOUT);

    // Visualize the final status.
    turnOffLEDs();
    if (HAS_SHELL)
        changeStateLED(RED, ON);
    else
        changeStateLED(BLUE, ON);

    return HAS_SHELL;
}

void waitForCompletingInitialization(void) {
    Thread *tp = chMsgWait();
    msg_t msg = chMsgGet(tp);
    chMsgRelease(tp, msg);
}

