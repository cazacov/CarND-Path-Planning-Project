#include "acceleration_profile.h"

double AccelerationProfile::get_s(double time) {
    return v * time + a*time*time / 2.0 + j * time * time * time / 6.0;
}

double AccelerationProfile::get_v(double time) {
    return v + a*time  + j * time * time;
}

double AccelerationProfile::get_a(double time) {
    return a + j * time;
}
