//
// Created by victor on 29.07.18.
//

#ifndef PATH_PLANNING_TRAJECTORYHELPER_H
#define PATH_PLANNING_TRAJECTORYHELPER_H

#include "spline.h"
#include <vector>
#include "trajectory.h"

class TrajectoryHelper {
private:
    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;

public:
    TrajectoryHelper(const std::vector<double> &_map_waypoints_x,
                const std::vector<double> &_map_waypoints_y,
                const std::vector<double> &_map_waypoints_s,
                const std::vector<double> &_map_waypoints_dx,
                const std::vector<double> &_map_waypoints_dy)
            : map_waypoints_x(_map_waypoints_x),
              map_waypoints_y(_map_waypoints_y),
              map_waypoints_s(_map_waypoints_s),
              map_waypoints_dx(_map_waypoints_dx),
              map_waypoints_dy(_map_waypoints_dy) {

    }

public:
    Trajectory buildTrajectory(double start_x, double start_y, double start_yaw, double s,
            const std::vector<double>& profile, int lane, double time);

    void generatePath(double start_x, double start_y, double start_yaw, const std::vector<double>& profile, double time, Trajectory& trajectory);

};


#endif //PATH_PLANNING_TRAJECTORYHELPER_H
