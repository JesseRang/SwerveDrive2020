#include "../include/SwerveMath.h"

SwerveMath::SwerveMath()
{
    this->L = WHEEL_BASE;
    this->W = TRACK_WIDTH;
    this->R = sqrt(pow(L,2) + pow(L,2));
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 2; j++)
        {
            WheelMatrix[i][j] = 0.0;
        }
    }
}

void SwerveMath::updateWheelMatrix(double Y_axis_left, double X_axis_left, double rotation_axis_right)
{
    this->A = X_axis_left - rotation_axis_right*(L/R);
    this->B = X_axis_left + rotation_axis_right*(L/R);
    this->C = -1*Y_axis_left - rotation_axis_right*(W/R);
    this->D = -1*Y_axis_left + rotation_axis_right*(W/R);

    /*Wheel Front-Right*/
    this->WheelMatrix[0][0] = sqrt(pow(B,2) + pow(C,2));
    this->WheelMatrix[0][1] = atan2(B,C)*(180/M_PI);
    /*Wheel Front-Left*/
    this->WheelMatrix[1][0] = sqrt(pow(B,2) + pow(D,2));
    this->WheelMatrix[1][1] = atan2(B,D)*(180/M_PI);
    /*wheel Back-Left*/
    this->WheelMatrix[2][0] = sqrt(pow(A,2) + pow(D,2));
    this->WheelMatrix[2][1] = atan2(A,D)*(180/M_PI);
    /*Wheel Back-Right*/
    this->WheelMatrix[3][0] = sqrt(pow(A,2) + pow(C,2));
    this->WheelMatrix[3][1] = atan2(A,C)*(180/M_PI);
}

void SwerveMath::normalizeWheelSpeeds()
{
    MAX = this->WheelMatrix[0][0];
    if(this->WheelMatrix[1][0] > MAX)
    {
        MAX = this->WheelMatrix[1][0];
    }
    if(this->WheelMatrix[2][0] > MAX)
    {
        MAX = this->WheelMatrix[2][0];
    }
    if(this->WheelMatrix[3][0] > MAX)
    {
        MAX = this->WheelMatrix[3][0];
    }

    /*set them to a value between 0 and 1*/
    if(MAX > 1)
    {
        this->WheelMatrix[0][0] /= MAX;
        this->WheelMatrix[1][0] /= MAX;
        this->WheelMatrix[2][0] /= MAX;
        this->WheelMatrix[3][0] /= MAX;
    }
}