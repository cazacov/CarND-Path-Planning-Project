#ifndef PATH_PLANNING_ACCELERATIONPROFILE_H
#define PATH_PLANNING_ACCELERATIONPROFILE_H


class AccelerationProfile {
public:
    double v;   // velocity
    double a;   // acceleration
    double j;   // jerk

    double get_s(double time);
    double get_v(double time);
    double get_a(double time);
};


#endif //PATH_PLANNING_ACCELERATIONPROFILE_H
