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

/**
 * Simplified implementation of sscanf supporting %d, %x, %c, %n:
 *
 * %d dec integer (e.g.: 42)
 * %x hex integer (e.g.: 0xa0)
 * %b bin integer (e.g.: b1010100010)
 * %n hex, dec or bin integer (e.g: 42, 0xa0, b1010100010)
 * %c any character
 */
int rsscanf(const char* str, const char* format, ...);

