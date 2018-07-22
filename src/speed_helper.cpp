#include "speed_helper.h"
#include "Eigen-3.3/Eigen/Dense"

using namespace Eigen;

vector<double> SpeedHelper::calculateAccelerationProfile(
        double start_speed, double start_acceleration,
        double target_speed, double time_frame,
        const double max_acceleration, const double max_jerk)
{
    double time_step = time_frame / 2;

    double delta_v = target_speed - start_speed;
    double a = (4 * delta_v / time_frame - start_acceleration) / 2;

    // Apply limits
    a = min(a, max_acceleration);
    a = max(a, -max_acceleration);

    a = min(a, start_acceleration + max_jerk * time_step);
    a = max(a, start_acceleration - max_jerk * time_step);

    a = min(a, max_jerk * time_step);
    a = max(a, -max_jerk * time_step);

    double delta_v1 = start_acceleration * time_step + (a - start_acceleration) * time_step / 2.0;
    double delta_v2 = a * time_step / 2.0;
    double final_speed = start_speed + delta_v1 + delta_v2;

    double jerk_1 =  (a - start_acceleration) / time_step;
    double jerk_2 = (0-a) / time_step;

    double distance_1 = start_speed * time_step
            + start_acceleration * time_step * time_step / 2.0
            + jerk_1 * time_step * time_step * time_step / 6.0;

    double distance_2 = (start_speed + delta_v1) * time_step
                        + a * time_step * time_step / 2.0
                        + jerk_2 * time_step * time_step * time_step / 6.0;


    return {final_speed, distance_1 + distance_2, a};

}

vector<double>
SpeedHelper::solveJmt(double start_speed, double final_speed, double start_acceleration, double time, double distance) {

    vector<double> start { 0, start_speed, start_acceleration};
    vector<double> end { distance, final_speed, 0};

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
