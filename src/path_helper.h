#ifndef PATH_PLANNING_PATHHELPER_H
#define PATH_PLANNING_PATHHELPER_H

#include <vector>

class PathHelper {
public:
    static void estimate_v_a(const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
        int min_tail_points, double &t_start_speed, double &t_start_acceleration);


    static void estimate_x_y_yaw(const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y, int min_tail_points,
                          double &t_start_x, double &t_start_y, double &t_start_yaw);

};


#endif //PATH_PLANNING_PATHHELPER_H
