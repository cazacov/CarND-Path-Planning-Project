#ifndef PATH_PLANNING_SPEED_HELPER_H
#define PATH_PLANNING_SPEED_HELPER_H

#include <vector>

using namespace std;

class SpeedHelper {
public:
    static vector<double> calculateAccelerationProfile(
            double start_speed, double start_acceleration,
            double target_speed, double time_frame,
            const double max_acceleration, const double max_jerk);

    static vector<double> solveJmt(double start_speed, double final_speed, double start_acceleration, double time, double distance);

    static vector<double> applyProfile(vector<double> profile, double time);
};

#endif //PATH_PLANNING_SPEED_HELPER_H
