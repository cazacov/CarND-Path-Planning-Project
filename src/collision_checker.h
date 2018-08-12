#ifndef PATH_PLANNING_COLLISIONCHECKER_H
#define PATH_PLANNING_COLLISIONCHECKER_H

#include <vector>
#include "trajectory.h"


class CollisionChecker {
private:
    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;

public:
    CollisionChecker(const std::vector<double> &_map_waypoints_x,
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


    bool check_collision(const std::vector<std::vector<double>> &sensor_fusion, double t_start_yaw, int current_lane,
                         Trajectory &trajectory, double start_time);

    bool check_limits(Trajectory &trajectory, const double max_speed, const double max_acceleration);

    double loop_distance(double s1, double s2);
};


#endif //PATH_PLANNING_COLLISIONCHECKER_H
