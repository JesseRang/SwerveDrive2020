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
#include <rev/CANSparkMax.h>

class Robot : public frc::TimedRobot {
  //set default PID coefficients
  double kP = 5e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000156, kMaxOutput = 1, kMinOutput = -1;
  //set default Smart Motion Coefficients
  double kMaxVel = 2000, kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;
  //set max motor RPM
  const double MaxRPM = 5700;
  /*swerve drive module locations*/
  frc::Translation2d m_frontLeftLocation{0.381_m, 0.381_m};
  frc::Translation2d m_frontRightLocation{0.381_m, -0.381_m};
  frc::Translation2d m_backLeftLocation{-0.381_m, 0.381_m};
  frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};
  /*Motors for going forwards and reverse*/
  rev::CANSparkMax driveMotor{0, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax driveMotor{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax driveMotor{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax driveMotor{3, rev::CANSparkMax::MotorType::kBrushless};
  /*Motors for rotating the wheels*/
  rev::CANSparkMax turnMotor{4, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax turnMotor{5, rev::CANSparkMax::MotorType::kBrushless}; 
  rev::CANSparkMax turnMotor{6, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax turnMotor{7, rev::CANSparkMax::MotorType::kBrushless};

/*--------------------------------------------------------------*/
/*Construct the SwerveMath object*/
SwerveMath swerveMath;
/*-------------------------------------------------------------*/

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
  /*creating kinematics swerve class*/
  frc::SwerveDriveKinematics<4> m_kinematics{
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
  m_backRightLocation};
  frc::ChassisSpeeds  speeds();
  /*Construct the Xbox or Joystick controller classes*/
  frc::Joystick driveController{0}; //0 is the port that the controller is at
/*-------------------------------------------------------------*/

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
