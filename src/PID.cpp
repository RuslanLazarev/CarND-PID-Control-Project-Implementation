#include "PID.h"
#include <vector>
#include <numeric>

using namespace std;


/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this -> Kp = Kp;
    this -> Ki = Ki;
    this -> Kd = Kd;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    iter = 0;
    best_err = 0.0;
    total_err = 0.0;

    vector<double> p[3];
    vector<double> dp[3];
    p << Kp, Ki, Kd;
    dp << 0.2, 0.5, 0,2;
    
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    //twiddleCorrect = (-p[0] * cte) + (-p[1] * i_error) + (-p[2] * (cte - d_error));
    
}

double PID::TotalError() {
    return Kp*p_error + Ki*i_error + Kd*d_error;
}

vector<double> PID::Twiddle(double cte) {
    best_err = total_err;
    sum_dp = accumulate(dp.begin(), dp.end(),0)
    while sum_dp > 0.2 {
        for (int i = 0, i < p.size(), i++) {
            p[i] += dp[i];
            total_err = TotalError(cte);

            if (total_err < best_err) {
                best_err = total_err;
                dp[i] *= 1.1;
            } else {
                p[i] -= 2 * dp[i];
                total_err = TotalError(cte);

                if (total_err < best_err) {
                    best_err = total_err;
                    dp[i] *= 1.1;
                } else {
                    p[i] += dp[i];
                    dp[i] *= 0.9;
                }

            }
            
        }
        
    } 

    return p;
}

double PID::ComputeSteer() {
    return -Kp*p_error - Ki*i_error - Kd*d_error;
    // In case of using twiddle:
    // std::vector<double> pSteer = pidSteer.Twiddle(double cte)
    // return -pSteer[0] * p_error - pSteer[1] * d_error - pSteer[2] * i_error
}

double PID::ComputeThrottle(double throttleMax) {
    return throttleMax -Kp*p_error - Ki*i_error - Kd*d_error;
}


