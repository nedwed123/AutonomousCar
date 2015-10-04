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

#ifndef USE_SHELL_H
#define USE_SHELL_H

#include "DiscoveryBoard.h"

/**
 * This method is invoked as the first call in newly created
 * threads to suspend the further execution unless the
 * initialization period with waiting for the user pressing
 * the button has passed.
 */
void waitForCompletingInitialization(void);

/**
 * @return 1 if the user wants to use the shell, 0 otherwise.
 */
int hasShell(void);

/**
 * This method is used to determine whether to start the
 * UI-shell or to transfer the data according to the protocol.
 *
 * @return 1 if the user wants to use the shell, 0 otherwise.
 */
int useShell(void);

#endif // USE_SHELL_H

