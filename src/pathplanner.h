#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include <vector>
#include "spline.h"

using namespace std;

class PathPlanner {
private:
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

public:
    PathPlanner(const std::vector<double>&  _map_waypoints_x,
                const std::vector<double>&  _map_waypoints_y,
                const std::vector<double>&  _map_waypoints_s,
                const std::vector<double>&  _map_waypoints_dx,
                const std::vector<double>&  _map_waypoints_dy);

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    void planPath(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, vector<vector<double>> sensor_fusion,
                      vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d);

    tk::spline buildTrajectory(double x, double y, double yaw, double s, vector<double> jmt, int lane, double time);

    double applyJmt(const vector<double>& coefficients, double time);

    void
    generatePath(double x, double y, double yaw, vector<double> jmt, tk::spline trajectory, unsigned long existing_points,
                     vector<double>& path_x, vector<double>& path_y, vector<double>& path_v, vector<double>& path_a);
};

#endif //PATH_PLANNING_PATHPLANNER_H
