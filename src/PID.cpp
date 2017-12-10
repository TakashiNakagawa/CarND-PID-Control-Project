#include "PID.h"


/*
* TODO: Complete the PID class.
*/

PID::PID()
{
//    Init(0, 0, 0);
}

PID::~PID() {}

void PID::Init(double p, double i, double d)
{
    Kp = p;
    Ki = i;
    Kd = d;
    p_error = 0;
    i_error = 0;
    d_error = 0;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() {
    return - Kp*p_error - Ki*i_error - Kd*d_error;
}


