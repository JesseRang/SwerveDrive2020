/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/math>
#include <frc/SmartDashboard/SmartDashboard.h>

double kP = .008 /*6e-5*/, kI = 1e-6, kD = 0, kIz = 0.001, kFF = 0 /*0.000015*/, kMaxOutput = 1.0, kMinOutput = -1.0;

SwerveModule::SwerveModule(const int driveMotorChannel, const int turningMotorChannel) : m_driveMotor{driveMotorChannel, brushless}, m_turningMotor{turningMotorChannel, brushless}
{
    m_turningEncoder.SetPosition(0);
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.SetPositionConversionFactor(2 * wpi::math::pi * kWheelRadius / kEncoderResolution);
    //set position conversion factor might not be the correct function to use here, we need to test and see what the robot does with this before changing

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    m_turningEncoder.SetPositionConversionFactor(1 / /*2 * wpi::math::pi */  18);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.

    frc::SmartDashboard::PutNumber("Drive P", kP);
    frc::SmartDashboard::PutNumber("Drive I", kI);
    frc::SmartDashboard::PutNumber("Drive D", kD);
    frc::SmartDashboard::PutNumber("Drive kIz", kIz);
    frc::SmartDashboard::PutNumber("Drive kFF", kFF);

    m_drivePIDController.SetP(frc::SmartDashboard::GetNumber("Drive P", kP));
    //m_drivePIDController.SetI(frc::SmartDashboard::GetNumber("Drive I", 0)); //kI
    //m_drivePIDController.SetD(frc::SmartDashboard::GetNumber("Drive D", 0)); //kD
    //m_drivePIDController.SetIZone(frc::SmartDashboard::GetNumber("Drive kIz", 0)); //kIz
    //m_drivePIDController.SetFF(frc::SmartDashboard::GetNumber("Drive kFF", 0)); //kFF
    //m_drivePIDController.SetOutputRange(kMinOutput, kMaxOutput);

    frc::SmartDashboard::PutNumber("Turn P", kP);
    frc::SmartDashboard::PutNumber("Turn I", kI);
    frc::SmartDashboard::PutNumber("Turn D", kD);
    frc::SmartDashboard::PutNumber("Turn kIz", kIz);
    frc::SmartDashboard::PutNumber("Turn kFF", kFF);

    m_turnPIDController.SetP(frc::SmartDashboard::GetNumber("Turn P", kP));
    m_turnPIDController.SetI(frc::SmartDashboard::GetNumber("Turn I", kI));
    m_turnPIDController.SetD(frc::SmartDashboard::GetNumber("Turn D", 0));
    m_turnPIDController.SetIZone(frc::SmartDashboard::GetNumber("Turn kIz", kIz));
    m_turnPIDController.SetFF(frc::SmartDashboard::GetNumber("Turn kFF", 0));
    m_turnPIDController.SetOutputRange(-1, 1); //used to be -pi and pi
}

//for the following function we need to get the velocity of the drive encoder and the rotations of the turning motor
frc::SwerveModuleState SwerveModule::GetState()
{
    return {units::meters_per_second_t{m_driveEncoder.GetVelocity()}, frc::Rotation2d(units::radian_t(m_turningEncoder.GetPosition()))};
}

/* double PrintSetAngle(const frc::SwerveModuleState &state)
{
    auto angle = state.angle.Radians();
    double doubleangle = angle.to<double>();
    return doubleangle * 100;
} */

void SwerveModule::SetDesiredState(const frc::SwerveModuleState &state)
{
    // Calculate the drive output from the drive PID controller.

    // Calculate the turning motor output from the turning PID controller.
    //const auto turnOutput = m_turnPIDController.Calculate(units::radian_t(m_turningEncoder.GetPosition()), state.angle.Radians());

    //const auto turnFeedforward = m_turnFeedforward.Calculate(m_turnPIDController.GetSetpoint().velocity);

    auto angle = state.angle.Radians();
    double doubleangle = angle.to<double>();

    // Set the motor outputs.
    //m_driveMotor.EnableVoltageCompensation(double {units::volt_t{driveOutput} + driveFeedforward});//SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
    //m_drivePIDController.SetReference(state.speed.to<double>() * 500, rev::ControlType::kVelocity);
    frc::SmartDashboard::PutNumber("Drive Speed", state.speed.to<double>() * 500);
    //get rotations of motor and multiply by radians to get double
    m_turnPIDController.SetReference((doubleangle / wpi::math::pi)* 21 /** 100 /** 500*/, rev::ControlType::kPosition);
    frc::SmartDashboard::PutNumber("Turn Set Reference", (doubleangle / wpi::math::pi) * 180);
    frc::SmartDashboard::PutNumber("Rotate Speed", doubleangle);
}
