#ifndef _SWERVEMATH_H_
#define _SWERVEMATH_H_
#define WHEEL_BASE 40.0
#define TRACK_WIDTH 40.0
#include <math.h>
#include <tgmath.h>
#include <bits/stdc++.h>

class SwerveMath
{
    double L;
    double W;
    double R;

    public:
        double WheelMatrix[4][2]; //this will contain wheel speeds and wheel angles
        double A;
        double B;
        double C;
        double D;
        double MAX;
        SwerveMath();
        void updateWheelMatrix(double Y_axis_left, double X_axis_left, double rotation_axis_right); //updates wheel speeds and angles
        void normalizeWheelSpeeds();

};

#endif