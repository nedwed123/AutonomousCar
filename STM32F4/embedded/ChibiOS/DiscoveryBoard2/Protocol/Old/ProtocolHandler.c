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

///////////////////////////////////////////////////////////////////////////////
// Data structures and configuation.
///////////////////////////////////////////////////////////////////////////////

// Configuration of the serial protocol between STM32F4 Discovery Board and PandaBoard ES.
const SerialConfig portConfig2 = {
    115000,
    0,
    USART_CR2_STOP1_BITS | USART_CR2_LINEN,
    USART_CR3_CTSE
};

// Buffer for the received data.
int receiveBuffer[3] = {0, 0, 0}; 

///////////////////////////////////////////////////////////////////////////////
// Interface methods.
///////////////////////////////////////////////////////////////////////////////

/**
 * This method parses the received data.
 *
 * @param str Pointer to received data.
 */
void parse(char *str) {
    char tmp[4][4];
    int len = strlen(str);
    int x=0, y=0, i=0, j=0;

    // Delete all data in tmp.
    for (i=0; i<4; i++)
        for (j=0; j<4; j++)
            tmp[i][j]=' ';

    // Parse data to array of string.
    for (i=0; i<len; i++) {
        if (str[i]!=',')
            tmp[x][y] = str[i];

        y++;	  
        if (str[i]==',') {
            tmp[x][y]='\0';
            x++;
            y=0; 
        }
    }

    // Convert small strings into array.
    for (i=0; i<3; i++)
        receiveBuffer[i] = atoi(tmp[i]);
}

void activityIndicator(void) {
    static int counter = 0;

    if (counter < 3) {
        changeStateLED(GREEN, ON);
    }
    if (counter >= 3 && counter < 6) {
        changeStateLED(GREEN, OFF);
    }
    if (counter >= 6) counter = 0;
    counter++;
}

void runProtocol(void) {
    int8_t accelData[2] = {0,0}; // Data from STM32F4 Discovery Board's onboard accelerometer.
    int ir_data[3] = {0,0,0}; // Data from the three connected infrared sensors. 
    int16_t us_data[3] = {0,0,0}; // Data from the three connected ultra sonic sensors.
    float imuData[7] = {0,0,0,0,0,0,0}; // Data from the IMU algorithms.
    int* razorInfo = NULL; // Pointer to the Razor 9DoF IMU board data (not implemented yet).
    uint8_t sentData[4] = {0,0,0,0}; // Buffer that contains the encoded data to be returned to the PandaBoard ES.

    char receivedInfo[12]; // Data structure that holds the received data from PandaBoard ES.

    systime_t tmo = MS2ST(10); // Receive time out.

    // Start the serial over USB driver in order to exchange data between STM32F4 Discovery Board and PandaBoard ES.
    sdStart((struct SerialDriver*)&SDU1,&portConfig2);

    while (TRUE) {
        // Indicate activity to the user.
        activityIndicator();

        // Update internal data structures.
        getOnboardAccelerometerData(accelData);
        getInfraredData(ir_data);
        getUltrasonicData(us_data);

        // Make sure that we actually have received some data (ie. the first byte of receivedInfo is changed).
        receivedInfo[0]='T';
        sdRead(&SDU1, (uint8_t*)receivedInfo, tmo);

        if (receivedInfo[0] != 'T'){
            // Parse the received data.
            receivedInfo[11]='\0';
            parse(receivedInfo);

            // Set the data values for the steering servo and the acceleration motor.
            setMotorData(receiveBuffer[1], 1550);

            // Encode our reply according to the requested data into sentData.
            translate(receiveBuffer[0], ir_data, us_data, razorInfo, imuData, accelData, sentData);

            // Reply to the PandaBoard ES.
            sdWrite(&SDU1, sentData, 4);
        }
    }
}

