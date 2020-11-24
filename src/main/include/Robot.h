/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
/*need to include the library for kinematics*/
#include <frc/kinematics/SwerveDriveKinematics.h>
/*include the JoyStick libraries or Xbox Controller*/
//#include <frc/XboxController.h>
//#include <frc/GenericHID.h>
#include <frc/Joystick.h>
#include "../include/SwerveMath.h"
#include "../include/Talon.h"

/*--------------------------------------------------------------*/
/*Construct the SwerveMath object*/
SwerveMath swerveMath;
/*-------------------------------------------------------------*/

/*Construct the Xbox or Joystick controller classes*/
frc::Joystick driveController{0}; //0 is the port that the controller is at
/*-------------------------------------------------------------*/

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
