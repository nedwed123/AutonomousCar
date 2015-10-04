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

int rsscanf(const char* str, const char* format, ...) {
    va_list ap;
    int value, tmp;
    int count;
    int pos;
    char neg, fmt_code;

    va_start(ap, format);

    for(count = 0; *format != 0 && *str != 0; format++, str++) {
        while (*format == ' ' && *format != 0)
            format++;
        if (*format == 0)
            break;

        while (*str == ' ' && *str != 0)
            str++;
        if (*str == 0)
            break;

        if (*format == '%') {
            format++;
            if (*format == 'n') {
                if (str[0] == '0' && (str[1] == 'x' || str[1] == 'X')) {
                    fmt_code = 'x';
                    str += 2;
                }
                else if (str[0] == 'b') {
                    fmt_code = 'b';
                    str++;
                }
                else
                    fmt_code = 'd';
            }
            else
                fmt_code = *format;

            switch (fmt_code) {
                case 'x':
                case 'X':
                    for (value = 0, pos = 0; *str != 0; str++, pos++) {
                        if ('0' <= *str && *str <= '9')
                            tmp = *str - '0';
                        else if ('a' <= *str && *str <= 'f')
                            tmp = *str - 'a' + 10;
                        else if ('A' <= *str && *str <= 'F')
                            tmp = *str - 'A' + 10;
                        else
                            break;

                        value *= 16;
                        value += tmp;
                    }
                    if (pos == 0)
                        return count;

                    *(va_arg(ap, int*)) = value;
                    count++;
                break;

                case 'b':
                    for (value = 0, pos = 0; *str != 0; str++, pos++) {
                        if (*str != '0' && *str != '1')
                            break;

                        value *= 2;
                        value += *str - '0';
                    }
                    if (pos == 0)
                        return count;

                    *(va_arg(ap, int*)) = value;
                    count++;
                break;

                case 'd':
                    if (*str == '-') {
                        neg = 1;
                        str++;
                    }
                    else
                        neg = 0;

                    for (value = 0, pos = 0; *str != 0; str++, pos++) {
                        if ('0' <= *str && *str <= '9')
                            value = value*10 + (int)(*str - '0');
                        else
                            break;
                    }
                    if (pos == 0)
                        return count;

                    *(va_arg(ap, int*)) = neg ? -value : value;
                    count++;
                break;

                case 'c':
                    *(va_arg(ap, char*)) = *str;
                    count++;
                break;

                default:
                return count;
            }
        }
        else {
            if (*format != *str)
                break;
        }
    }

    va_end(ap);

    return count;
}

