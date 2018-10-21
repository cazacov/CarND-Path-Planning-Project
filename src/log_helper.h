#ifndef PATH_PLANNING_LOGGER_H
#define PATH_PLANNING_LOGGER_H


#include "trajectory.h"
#include <vector>

class LogHelper
{
public:
    static void
    change_lane(double car_s, double car_d, double car_x, double car_y, double start_s, double start_d, double start_x,
                    double start_y, double start_speed, double start_acceleration, double start_yaw,
                    std::vector<std::vector<double>> &sensor_fusion, Trajectory &trajectory, int i, int i1);
};


#endif //PATH_PLANNING_LOGGER_H
