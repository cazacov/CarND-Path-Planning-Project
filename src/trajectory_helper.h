//
// Created by victor on 29.07.18.
//

#ifndef PATH_PLANNING_TRAJECTORYHELPER_H
#define PATH_PLANNING_TRAJECTORYHELPER_H

#include "spline.h"
#include "trajectory.h"

class TrajectoryHelper {
public:
    static Trajectory buildTrajectory(double start_x, double start_y, double start_yaw, double s,
            const std::vector<double> profile, int lane, double time,
                                      const std::vector<double>& map_waypoints_x,
                                      const std::vector<double>& map_waypoints_y,
                                      const std::vector<double>& map_waypoints_s);

    static void generatePath(double start_x, double start_y, double start_yaw, const std::vector<double>& profile, double time, Trajectory& trajectory);

};


#endif //PATH_PLANNING_TRAJECTORYHELPER_H
