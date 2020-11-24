#ifndef _SPARKMAX_H_
#define _SPARKMAX_H_
#include <rev/CANSparkMax.h>
//set default PID coefficients
double kP = 5e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000156, kMaxOutput = 1, kMinOutput = -1;
//set default Smart Motion Coefficients
double kMaxVel = 2000, kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;
//set max motor RPM
const double MaxRPM = 5700;

rev::CANSparkMax driveMotor{0, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax driveMotor{1, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax driveMotor{2, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax driveMotor{3, rev::CANSparkMax::MotorType::kBrushless};

rev::CANSparkMax turnMotor{4, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax turnMotor{5, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax turnMotor{6, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax turnMotor{7, rev::CANSparkMax::MotorType::kBrushless};

#endif