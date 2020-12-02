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
#define brushless rev::CANSparkMax::MotorType::kBrushless

class Robot : public frc::TimedRobot {
  //set default PID coefficients
  double kP = 5e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000156, kMaxOutput = 1, kMinOutput = -1;
  //set default Smart Motion Coefficients
  double kMaxVel = 2000, kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;
  //set max motor RPM
  const double MaxRPM = 5700;
  /*swerve drive module locations*/

  /*Motors for going forwards and reverse*/
  
  rev::CANSparkMax flMotor{0, brushless};
  rev::CANPIDController flPidController = flMotor.GetPIDController();
  rev::CANEncoder flEncoder = flMotor.GetEncoder();

  rev::CANSparkMax frMotor{1, brushless};
  rev::CANSparkMax blMotor{2, brushless};
  rev::CANSparkMax brMotor{3, brushless};
  /*Motors for rotating the wheels*/
  rev::CANSparkMax frtMotor{4, brushless};
  rev::CANSparkMax fltMotor{5, brushless}; 
  rev::CANSparkMax brtMotor{6, brushless};
  rev::CANSparkMax bltMotor{7, brushless};

/*--------------------------------------------------------------*/
/*Construct the SwerveMath object*/
  SwerveMath swerveMath = SwerveMath();
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

  frc::SwerveModuleState frontLeftState{0_mps, frc::Rotation2d(0_deg)};
  frc::SwerveModuleState frontRightState{0_mps, frc::Rotation2d(0_deg)};
  frc::SwerveModuleState backLeftState{0_mps, frc::Rotation2d(0_deg)};
  frc::SwerveModuleState backRightState{0_mps, frc::Rotation2d(0_deg)};

  /*Construct the Xbox or Joystick controller classes*/
  frc::Joystick driveController{0}; //0 is the port that the controller is at
/*-------------------------------------------------------------*/

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
