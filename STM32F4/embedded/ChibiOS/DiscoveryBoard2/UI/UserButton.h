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

#ifndef USER_BUTTON_H
#define USER_BUTTON_H

/**
 * This method waits timeOut seconds and checks
 * periodically if the user has pressed the button.
 *
 * @timeOut Time out for checking the user button.
 * @return 0 if the user has not pushed the button, 1 otherwise.
 */
int isUserButtonPressed(unsigned int timeOut);

#endif // USER_BUTTON_H

