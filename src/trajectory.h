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
    std::vector<double> path_a;
};


#endif //PATH_PLANNING_TRAJECTORY_H
