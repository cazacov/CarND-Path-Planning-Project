//
// Created by victor on 16.07.18.
//
#include <math.h>
#include <iostream>
#include <iomanip>
#include "pathplanner.h"
#include "maptransformer.h"

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

    cout    << "s=" << setw(6) << s
            << "\t"
            << "d=" << setw(6) << d << endl;


    double dist_inc = 0.3;
    for(int i = 0; i < 50; i++)
    {
        double new_s = s + dist_inc * (i+1);
        double new_d = 6;

        vector<double> xy =  MapTransformer::getXY(new_s, new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }

}
