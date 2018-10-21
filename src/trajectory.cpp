#include "trajectory.h"
#include <math.h>
using namespace std;


/*
 * Calculate max acceleration
 */
void Trajectory::update_metrics(double time_step, double initial_v, double start_x, double start_y) {

    // calculate tangential velocity
    v_tan.clear();
    for (int i = 0; i < path_x.size(); i++) {
        double dx;
        double dy;
        if (i > 0)
        {
            dx = path_x[i] - path_x[i-1];
            dy = path_y[i] - path_y[i-1];
        }
        else {
            dx = path_x[i] - start_x;
            dy = path_y[i] - start_y;
        }

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

    // Check first 20 points only
    for (int i = 0; i < 20; i++) {
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
