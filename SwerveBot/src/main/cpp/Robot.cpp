/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/SlewRateLimiter.h>
#include <frc/TimedRobot.h>
#include <frc/SmartDashboard/SmartDashboard.h>
//#include <frc/XboxController.h>
#include <frc/Joystick.h>

#include "Drivetrain.h"

class Robot : public frc::TimedRobot {
 public:
  void AutonomousPeriodic() override {
    DriveWithJoystick(false);
    m_swerve.UpdateOdometry();
  }

  void TeleopPeriodic() override { DriveWithJoystick(true); }

 private:
  //frc::XboxController m_controller{0};
  frc::Joystick m_controller{0};

  //joystick defines
  double leftX{m_controller.GetRawAxis(0)};
  double leftY{m_controller.GetRawAxis(1)};
  double rightX{m_controller.GetRawAxis(4)};

  double deadzone = 0.05;

  Drivetrain m_swerve;
  units::velocity::meters_per_second_t prev_x_speed;
  units::velocity::meters_per_second_t prev_y_speed;
  units::angular_velocity::radians_per_second_t prev_rot_speed;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{1 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{1 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{1 / 1_s};

  void DriveWithJoystick(bool fieldRelative) {
    
   leftX = m_controller.GetRawAxis(0);
   leftY = m_controller.GetRawAxis(1);
   rightX = m_controller.GetRawAxis(4);

  if(leftY < deadzone && leftY > (deadzone * -1)){
      leftY = 0;
  }

  if(leftX < deadzone && leftX > (deadzone * -1)){
      leftX = 0;
  }

  if(rightX < deadzone && rightX > (deadzone * -1)){
      rightX = 0;
  }
  
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    auto xSpeed = -m_xspeedLimiter.Calculate(leftX) * Drivetrain::kMaxSpeed;
    
    frc::SmartDashboard::PutNumber("leftY", leftY);
    frc::SmartDashboard::PutNumber("leftX", leftX);
    frc::SmartDashboard::PutNumber("rightX", rightX);

    frc::SmartDashboard::PutNumber("xSpeed", xSpeed.to<double>());
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    auto ySpeed = -m_yspeedLimiter.Calculate(leftY) * Drivetrain::kMaxSpeed;
    frc::SmartDashboard::PutNumber("ySpeed", ySpeed.to<double>());
    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    auto rot = -m_rotLimiter.Calculate(rightX) * Drivetrain::kMaxAngularSpeed;
    
    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative);
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
