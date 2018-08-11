#include "speed_helper.h"

AccelerationProfile SpeedHelper::calculateAccelerationProfile(
        double start_speed, double start_acceleration,
        double target_speed, double time_frame,
        const double max_acceleration, const double max_jerk)
{
    double delta_v = target_speed - start_speed;

    double a = delta_v / time_frame;
    a = min(a, max_acceleration);
    a = max(a, -max_acceleration);

    AccelerationProfile result{};

    result.v = start_speed;
    result.a = a;
    result.j = 0;   // jerk

    return result;
}

