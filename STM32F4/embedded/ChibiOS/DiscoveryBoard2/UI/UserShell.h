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

#ifndef USER_SHELL_H
#define USER_SHELL_H

#include "DiscoveryBoard.h"

// Commands that are available for the shell.
static const ShellCommand commands[] = {
    {"mem", commandMem},
    {"threads", commandThreads},
    {"printAccelerometer", commandPrintAccelerometer},
    {"pa", commandPrintAccelerometer},
    {"printTemperature", commandPrintTemperature},
    {"pt", commandPrintTemperature},
    {"printInfraredDistances", commandPrintInfraredDistances},
    {"pdi", commandPrintInfraredDistances},
    {"printUltrasonicDistances", commandPrintUltrasonicDistances},
    {"pdu", commandPrintUltrasonicDistances},
    {"setUltrasonicAddress", commandSetUltrasonicAddress},
    {"sua", commandSetUltrasonicAddress},
    {"triggerSingleUltrasonicMeasurement", commandTriggerSingleUltrasonicMeasurement},
    {"tsum", commandTriggerSingleUltrasonicMeasurement},
    {"startContinuousUltrasonicMeasurements", commandStartContinuousUltrasonicMeasurements},
    {"scum", commandStartContinuousUltrasonicMeasurements},
    {"stopContinuousUltrasonicMeasurements", commandStopContinuousUltrasonicMeasurements},
    {"scumoff", commandStopContinuousUltrasonicMeasurements},
    {"controlSteeringAccelerationMotors", commandControlSteeringAccelerationMotors},
    {"csam", commandControlSteeringAccelerationMotors},
    {"printRCReceiver", commandPrintRCReceiver},
    {"prc", commandPrintRCReceiver},
    {"printWheelEncoder", commandPrintWheelEncoder},
    {"pwe", commandPrintWheelEncoder},
    {"resetWheelEncoder", commandResetWheelEncoder},
    {"rwe", commandResetWheelEncoder},
//    {"printRazor9DoFIMU", commandPrintRazor9DoFIMU},
//    {"pr", commandPrintRazor9DoFIMU},
    {"rpn", commandRPNCalculator},
    {NULL, NULL}
};

/**
 * This method spawns an interactive user shell from
 * within an infinite loop (thus, this method will
 * not return!).
 */
int spawnShell(void);

#endif // USER_SHELL_H

