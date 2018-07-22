#ifndef PATH_PLANNING_SPEED_HELPER_H
#define PATH_PLANNING_SPEED_HELPER_H

#include <vector>

using namespace std;

class SpeedHelper {
public:
    static double calculateMaxSpeed(double start_speed, double start_acceleration, double time_frame, const double max_acceleration,
                                        double d);
    static vector<double>
    solveJmt(double start_speed, double final_speed, double start_acceleration, double time, double d);
};

#endif //PATH_PLANNING_SPEED_HELPER_H
