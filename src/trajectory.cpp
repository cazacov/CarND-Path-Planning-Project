//
// Created by victor on 29.07.18.
//

#include "trajectory.h"
#include <math.h>
using namespace std;

void Trajectory::update_metrics(double time_step) {

    // calculate tangential velocity
    v_tan.clear();
    v_tan.push_back(path_v[0]);
    for (int i = 0; i < path_x.size() - 1; i++) {
        double dx = path_x[i + 1] - path_x[i];
        double dy = path_y[i + 1] - path_y[i];
        double v = sqrt(dx * dx + dy * dy) / time_step;
        v_tan.push_back(v);
    }

    // calculate tangential acceleration
    vector<double> a_tan;
    for (int i = 0; i < v_tan.size() - 1; i++) {
        double v1 = v_tan[i];
        double v2 = v_tan[i + 1];
        double a = fabs(v2 - v1) / time_step;
        a_tan.push_back(a);
    }
    if (a_tan.size() > 3) {
        a_tan.insert(a_tan.begin(), (a_tan[1] + a_tan[2]) / 2);
    } else {
        a_tan.insert(a_tan.begin(), a_tan.front());
    }

    // calculate normal acceleration
    vector<double> a_norm;
    for (int i = 0; i < v_tan.size(); i++) {
        double acc_norm = v_tan[i] * v_tan[i] * path_k[i];
        a_norm.push_back(acc_norm);
    }

    vector<double> a_total_sum;
    a_total_sum.push_back(0);
    double a_sum = 0;
    for (int i = 0; i < a_tan.size(); i++) {
        double at = a_tan[i];
        double an = a_norm[i];
        double a_total = sqrt(at * at + an * an);
        a_sum += a_total;
        a_total_sum.push_back(a_sum);
    }

    // Calculate sliding averages and select max value
    max_acceleration = 0;
    int window_size = 25;   // 0.5 second

    for (int i = 0; i < a_total_sum.size() - window_size; i++) {
        double average = (a_total_sum[i + window_size] - a_total_sum[i]) / window_size;
        if (average > max_acceleration) {
            max_acceleration = average;
        }
    }
}
