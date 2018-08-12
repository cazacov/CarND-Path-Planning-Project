#ifndef PATH_PLANNING_SPEED_HELPER_H
#define PATH_PLANNING_SPEED_HELPER_H

#include <vector>
#include "acceleration_profile.h"

using namespace std;

class SpeedHelper {
public:
    static AccelerationProfile calculateAccelerationProfile(
            double start_speed, double start_acceleration,
            double target_speed, double time_frame,
            const double max_acceleration, const double max_jerk);

    static void estimate_v_a(const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
                             int min_tail_points, double &t_start_speed, double &t_start_acceleration);


    static void estimate_x_y_yaw(const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y, int min_tail_points,
                                 double &t_start_x, double &t_start_y, double &t_start_yaw);

};

#endif //PATH_PLANNING_SPEED_HELPER_H
