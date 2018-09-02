#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
    double p_error;
    double i_error;
    double d_error;

    double Kp;
    double Ki;
    double Kd;



}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this -> double Kp;
    this -> double Ki;
    this -> double Kd;

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
    return Kp*p_error + Ki*i_error + Kd*d_error;
}

