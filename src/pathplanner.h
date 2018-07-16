#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include <vector>

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
};

#endif //PATH_PLANNING_PATHPLANNER_H
