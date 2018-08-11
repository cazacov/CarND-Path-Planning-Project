#include "trajectory.h"
#include <math.h>
using namespace std;

void Trajectory::update_metrics(double time_step, double initial_v) {

    // calculate tangential velocity
    v_tan.clear();
    v_tan.push_back(initial_v);
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
    a_tan.push_back(a_tan.back());  // Duplicate last point ti get same number of items in all vectors

    // calculate normal acceleration
    vector<double> a_norm;
    for (int i = 0; i < v_tan.size(); i++) {
        double acc_norm = v_tan[i] * v_tan[i] * path_k[i];
        a_norm.push_back(acc_norm);
    }

    // prepare sum vectors for sliding average calculation
    vector<double> a_tan_sum;
    double a_t_s = 0;
    for (int i = 0; i < a_tan.size(); i++)
    {
        a_t_s += a_tan[i];
        a_tan_sum.push_back(a_t_s);
    }

    vector<double> a_norm_sum;
    double a_n_s = 0;
    for (int i = 0; i < a_norm.size(); i++)
    {
        a_n_s += a_norm[i];
        a_norm_sum.push_back(a_n_s);
    }

    // Calculate sliding averages and select max value
    max_tan_acceleration = 0;
    max_norm_acceleration = 0;
    max_total_acceleration = 0;

    int window_size = 10;   // 0.2 second

    for (int i = 0; i < a_tan_sum.size() - window_size; i++) {
        double average_a_tan = (a_tan_sum[i + window_size] - a_tan_sum[i]) / window_size;
        double average_a_norm = (a_norm_sum[i + window_size] - a_norm_sum[i]) / window_size;
        double average_a_total = sqrt(average_a_norm * average_a_norm + average_a_tan * average_a_tan);

        if (average_a_total > max_total_acceleration) {
            max_total_acceleration = average_a_total;
        }
        if (average_a_tan > max_tan_acceleration) {
            max_tan_acceleration = average_a_tan;
        }
        if (average_a_norm > max_norm_acceleration) {
            max_norm_acceleration = average_a_norm;
        }
    }
}
