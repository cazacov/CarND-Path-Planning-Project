#include <math.h>
#include <iostream>
#include <iomanip>
#include "Eigen-3.3/Eigen/Dense"

#include "pathplanner.h"
#include "maptransformer.h"
#include "math_helper.h"

PathPlanner::PathPlanner(const std::vector<double> & _map_waypoints_x,
                         const std::vector<double> & _map_waypoints_y,
                         const std::vector<double> & _map_waypoints_s,
                         const std::vector<double> & _map_waypoints_dx,
                         const std::vector<double> & _map_waypoints_dy)
    : map_waypoints_x(_map_waypoints_x),
      map_waypoints_y(_map_waypoints_y),
      map_waypoints_s(_map_waypoints_s),
      map_waypoints_dx(_map_waypoints_dx),
      map_waypoints_dy(_map_waypoints_dy)
{

}

const double kMaxAcceleration = 5; // m/s^2
const double kMaxJerk =         5; // m/s^3



void
PathPlanner::planPath(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
                      vector<vector<double>> sensor_fusion,
                      vector<double> previous_path_x, vector<double> previous_path_y,
                      double end_path_s, double end_path_d) {
    next_x_vals.clear();
    next_y_vals.clear();


    vector<double> frenet = MapTransformer::getFrenet(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);

    double s = frenet[0];
    double d = frenet[1];



    int target_lane = 1;

    // Starting point of new trajectory
    double t_start_x = car_x;
    double t_start_y = car_y;
    double t_start_yaw = car_yaw;
    double t_start_speed = car_speed;

    double t_start_speed2 = car_speed;
    double t_start_accel = 0;

    const int min_tail_points = 10;


    if (previous_path_x.size() >= min_tail_points) // we have some data from previous trajectory
    {
        // estimate car speed and yaw
        double x_back = previous_path_x.back();
        double y_back = previous_path_y.back();
        double x_prev = *(previous_path_x.end() - min_tail_points);  // -1 because the end() is referring past-the-end element
        double y_prev = *(previous_path_y.end() - min_tail_points);

        t_start_speed = MapTransformer::distance(x_back,y_back, x_prev, y_prev) / ((min_tail_points - 1) * 0.02);
        t_start_yaw = atan2(y_back - y_prev, x_back - x_prev);


        // Use polynomial fitting to estimate speed and acceleration at last min_tail_points of the previous path
        vector<double> poly_x;
        vector<double>  poly_y;
        int start_idx = previous_path_x.size() - min_tail_points;
        for (int i = start_idx; i <  previous_path_x.size(); i++)
        {
            poly_x.push_back(i);
            poly_y.push_back(MapTransformer::distance(previous_path_x[start_idx], previous_path_y[start_idx],previous_path_x[i], previous_path_y[i]));
        }

        vector<double> coeff;
        MathHelper::polyfit(poly_x, poly_y, coeff, 2);

        t_start_speed2 = coeff[1] / 0.02;
        t_start_accel = coeff[2] / (0.02 * 0.02);

    }

    cout    << "s=" << setw(6) << s
            << "\t"
            << "d=" << setw(6) << d
            << "\t"
            << "speed=" << setw(6) << t_start_speed
            << "\t"
            << "speed2=" << setw(6) << t_start_speed2
            << "\t"
            << "accel=" << setw(6) << t_start_accel
            << endl;

    double dist_inc = 0.4;
    for(int i = 0; i < 50; i++)
    {
        double new_s = car_s + dist_inc * (i+1);
        double new_d = target_lane * 4.0 + 2.0;

        vector<double> xy =  MapTransformer::getXY(new_s, new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }

}
