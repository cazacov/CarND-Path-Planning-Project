//
// Created by victor on 29.07.18.
//

#include "path_helper.h"
#include "maptransformer.h"
#include "math_helper.h"

#include <vector>

using namespace std;

void PathHelper::estimate_v_a(const vector<double> &previous_path_x, const vector<double> &previous_path_y,
                               const int min_tail_points, double &t_start_speed, double &t_start_acceleration)
{

    // Use polynomial fitting to estimate speed and acceleration at last min_tail_points of the previous path

    vector<double> poly_x;
    vector<double>  poly_y;
    int start_idx = previous_path_x.size() - min_tail_points;
    for (int i = 0; i <  min_tail_points; i++)
    {
        poly_x.push_back(i * 0.02);
        poly_y.push_back(MapTransformer::distance(
                previous_path_x[start_idx], previous_path_y[start_idx], previous_path_x[start_idx + i], previous_path_y[start_idx + i]));
    }

    // Approximate points with 2nd order polynomial
    vector<double> coefficients;
    MathHelper::polyfit(poly_x, poly_y, coefficients, 2);

    double v0 = coefficients[1];
    double a0 = coefficients[2] * 2;

    double t = 0.02 * (min_tail_points - 1);

    t_start_acceleration = a0;
    t_start_speed =  v0 +  a0 * t;  // speed at the last point of previous path
}

void
PathHelper::estimate_x_y_yaw(const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
                             const int min_tail_points, double &t_start_x, double &t_start_y, double &t_start_yaw) {

        double x_back = previous_path_x.back();
        double y_back = previous_path_y.back();
        double x_prev = *(previous_path_x.end() - min_tail_points);
        double y_prev = *(previous_path_y.end() - min_tail_points);

        t_start_x = x_back;
        t_start_y = y_back;
        t_start_yaw = atan2(y_back - y_prev, x_back - x_prev);
    }

