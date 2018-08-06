//
// Created by victor on 29.07.18.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>

#include "spline.h"


class Trajectory {
public:
    tk::spline spline;
    std::vector<double> path_x;
    std::vector<double> path_y;
    std::vector<double> path_v;
    std::vector<double> path_v_p;
    std::vector<double> path_a;
    std::vector<double> path_c; // point curvature
    std::vector<double> path_k; // spline curvature

    double collision_my_s;
    double collision_my_d;
    double collision_other_s;
    double collision_other_d;
    double collision_time;
    int collision_other_id;

    void update_metrics(double time_step);

    double max_speed;
    double mean_normal_acceleration;
    double max_normal_acceleration;
};


#endif //PATH_PLANNING_TRAJECTORY_H
