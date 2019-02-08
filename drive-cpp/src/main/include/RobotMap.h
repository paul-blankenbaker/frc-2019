/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

// Numeric constants missing from C++ language
constexpr auto Math_PI = 3.14159265358979323846;

// Map of CAN ID allocation
constexpr int kCanDriveLeft0 = 10;
constexpr int kCanDriveLeft1 = 11;
constexpr int kCanDriveLeft2 = 12;

constexpr int kCanDriveRight0 = 20;
constexpr int kCanDriveRight1 = 21;
constexpr int kCanDriveRight2 = 22;

// Map of DIO allocation
constexpr int kDioDriveLeftEncA = 0;
constexpr int kDioDriveLeftEncB = 1;
constexpr int kDioDriveRightEncA = 2;
constexpr int kDioDriveRightEncB = 3;
