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

// Configuration of the shell.
static const ShellConfig shellConfiguration = {
  (BaseSequentialStream *)&SDU1,
  commands
};

int spawnShell(void) {
    const int ONE_THOUSAND_MILLISECONDS = 1000;
    const int THREAD_WORKING_AREA_SIZE = 2048;

    Thread *shellThread = NULL;

    while (TRUE) {
        // If we don't have a shell but a living USB connection, create a new thread with the shell.
        if (!shellThread && isUSBActive()) {
            shellThread = shellCreate(&shellConfiguration, THD_WA_SIZE(THREAD_WORKING_AREA_SIZE), NORMALPRIO);
        }
        else if (chThdTerminated(shellThread)) {
            // Recover memory of the previous shell.
            chThdRelease(shellThread);

            // Trigger spawning of a new shell.
            shellThread = NULL;
        }

        chThdSleepMilliseconds(ONE_THOUSAND_MILLISECONDS);
    }
}

