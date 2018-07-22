#include "speed_helper.h"
#include "Eigen-3.3/Eigen/Dense"

using namespace Eigen;

double SpeedHelper::calculateMaxSpeed(double start_speed, double start_acceleration, double time_frame, const double max_acceleration)
{
    double jerk = (max_acceleration * 2 - start_acceleration) / time_frame;
    double t2 = (time_frame + start_acceleration / jerk) / 2.0;
    double t1 = time_frame - t2;

    // accelerate from start_acceleration to max_acceleration
    double max_v = start_speed + start_acceleration * t1 + (jerk * t1 * t1)/2.0;

    // continue increasing speed reducing acceleration form max_acceleration to 0
    max_v += (start_acceleration + jerk * t1) * t2 - jerk * t2 * t2 / 2.0;
    return max_v;
}

vector<double>
SpeedHelper::solveJmt(double start_s, double start_speed, double final_speed, double start_acceleration, double time) {

    double final_s = start_s + (start_speed + final_speed) / 2 * time;

    vector<double> start { start_s, start_speed, start_acceleration};
    vector<double> end { final_s, final_speed, 0};

    double t2 = time*time;
    double t3 = t2*time;
    double t4 = t3*time;
    double t5 = t4*time;

    Matrix3d A;
    A << t3, t4, t5,
            3*t2, 4*t3, 5*t4,
            6*time, 12*t2,  20*t3;

    Vector3d B;

    B << end[0] - (start[0] + start[1]*time + 1 / 2.0 * start[2] * t2),
            end[1] - (start[1] + start[2] * time),
            end[2] - start[2];


    Vector3d x = A.colPivHouseholderQr().solve(B);
    return {start[0], start[1], start[2]/2.0, x[0], x[1], x[2]};
}
