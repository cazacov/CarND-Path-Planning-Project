//
// Created by victor on 29.07.18.
//

#ifndef PATH_PLANNING_TRAJECTORYHELPER_H
#define PATH_PLANNING_TRAJECTORYHELPER_H

#include "spline.h"

class TrajectoryHelper {
public:
    static tk::spline buildTrajectory(double start_x, double start_y, double start_yaw, double s,
            const std::vector<double> profile, int lane, double time,
                                      const std::vector<double>& map_waypoints_x,
                                      const std::vector<double>& map_waypoints_y,
                                      const std::vector<double>& map_waypoints_s);

    static void
    generatePath(double start_x, double start_y, double start_yaw, std::vector<double> profile,
                 tk::spline trajectory, double time,
                 std::vector<double>& path_x, std::vector<double>& path_y, std::vector<double>& path_v, std::vector<double>& path_a);

};


#endif //PATH_PLANNING_TRAJECTORYHELPER_H
