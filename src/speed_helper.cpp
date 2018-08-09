#include "speed_helper.h"
#include "Eigen-3.3/Eigen/Dense"

using namespace Eigen;

AccelerationProfile SpeedHelper::calculateAccelerationProfile(
        double start_speed, double start_acceleration,
        double target_speed, double time_frame,
        const double max_acceleration, const double max_jerk)
{
    double time_step = time_frame / 2;
    double delta_v = target_speed - start_speed;

/*
    double a0 = start_acceleration;
    double a = (4 * delta_v / time_frame - start_acceleration) / 2;

    // Apply limits
    a = min(a, max_acceleration);
    a = max(a, -max_acceleration);

    a = min(a, start_acceleration + max_jerk * time_step);
    a = max(a, start_acceleration - max_jerk * time_step);

    a = min(a, max_jerk * time_step);
    a = max(a, -max_jerk * time_step);

    double jerk_1 =  (a - start_acceleration) / time_step;
    double jerk_2 = (0-a) / time_step;

    double delta_v1 = start_acceleration * time_step + (a - start_acceleration) * time_step / 2.0;
    double delta_v2 = a * time_step / 2.0;
*/
    double a = delta_v / time_frame;
    a = min(a, max_acceleration);
    a = max(a, -max_acceleration);

    double a0 = a;                      // ignore previous acceleration because it cannot be measured precisely
    double delta_v1 = a * time_step;
    double delta_v2 = a * time_step;
    double jerk_1 = 0;
    double jerk_2 = 0;

    double final_speed = start_speed + delta_v1 + delta_v2;

    double distance_1 = start_speed * time_step
            + a0 * time_step * time_step / 2.0
            + jerk_1 * time_step * time_step * time_step / 6.0;

    double distance_2 = (start_speed + delta_v1) * time_step
                        + a * time_step * time_step / 2.0
                        + jerk_2 * time_step * time_step * time_step / 6.0;

    AccelerationProfile result;

    result.v = start_speed;
    result.a = a0;
    result.j = jerk_1;

    return result;

/*
    {
        final_speed,
        distance_1 + distance_2,
        a,
        time_step,
        start_speed,
        a0,
        jerk_1,
        (start_speed + delta_v1),
        a,
        jerk_2,
        final_speed,
        0,
        0
    };
*/
}

