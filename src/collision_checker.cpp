#include <math.h>

#include "collision_checker.h"
#include "maptransformer.h"

#include <iostream>
#include <iomanip>


using namespace std;

bool CollisionChecker::check_collision(const vector<vector<double>> &sensor_fusion, double t_start_yaw,
                                       Trajectory &trajectory, double start_time) {
    bool collision_detected = false;

    trajectory.my_sd.clear();
    trajectory.cars_sd.clear();

    for (int i = 0; i < trajectory.path_x.size(); i++) {

        vector<double> carsd;

        double delta_time = (i + 1) * 0.02;
        double time = start_time + delta_time;
        double my_x = trajectory.path_x[i];
        double my_y = trajectory.path_y[i];
        double my_yaw = t_start_yaw;
        if (i > 2) {
            my_yaw = atan2(trajectory.path_y[i] - trajectory.path_y[i - 2],
                           trajectory.path_x[i] - trajectory.path_x[i - 2]);
        }
        vector<double> my_frenet = MapTransformer::getFrenet(my_x, my_y, my_yaw, map_waypoints_x, map_waypoints_y);
        double my_s = my_frenet[0];
        double my_d = my_frenet[1];
        int my_lane = MapTransformer::d2lane(my_d);

        trajectory.my_sd.push_back({my_s, my_d, trajectory.path_v[i]});

        for (int car = 0; car < sensor_fusion.size(); car++) {
            int car_id  = sensor_fusion[car][0];
            double car_x = sensor_fusion[car][1];
            double car_y = sensor_fusion[car][2];
            double car_vx = sensor_fusion[car][3];
            double car_vy = sensor_fusion[car][4];
            double car_v = sqrt(car_vx * car_vx + car_vy * car_vy);

            double car_s0 = sensor_fusion[car][5];
            double car_s = car_s0 + time * car_v;
            double car_d = sensor_fusion[car][6];
            int car_lane = MapTransformer::d2lane(car_d);

            carsd.push_back(car_s);
            carsd.push_back(car_d);

            // round car's d coordinate
            car_d = MapTransformer::lane2d(car_lane);

            if (loop_distance(my_s, car_s0) > 10 && car_lane == trajectory.start_lane)
            {
                // Ignore cars behind me. They should keep safe distance.
                continue;
            }

            double car_yaw = atan2(car_vy, car_vx);
            vector<double> car_xy = MapTransformer::getXY(car_s, car_d, map_waypoints_s, map_waypoints_x,
                                                          map_waypoints_y);

            bool is_changing_lane = trajectory.start_lane != trajectory.target_lane;

            double min_s_distance = trajectory.path_v[i] * 1.5;   // At least 1.5 second distance to next car
            double min_back_s = 0;   // Ignore cars behind us
            double min_d_distance = 2.0;

            if (is_changing_lane)
            {
                min_d_distance = 2.6;   // more defensive collision check

                if (car_lane == trajectory.start_lane)
                {
                    min_s_distance = trajectory.path_v[i] * 1; // I can pass closer to another car in my lane when changing to another lane
                }
                else {
                    min_s_distance = trajectory.path_v[i] * 2; // add penalty for changing lane
                    min_back_s = 12.5; // Check if there are cars behind close to us
                }
            }

            if ( loop_distance(car_s, my_s) > -min_back_s && loop_distance(car_s, my_s) < min_s_distance) {
                if (fabs(my_d - car_d) < min_d_distance) {

                    trajectory.collision_my_s = my_s;
                    trajectory.collision_my_d = my_d;
                    trajectory.collision_other_s = car_s;
                    trajectory.collision_other_d = car_d;
                    trajectory.collision_other_car_id = car_id;
                    trajectory.collision_time = time;
                    collision_detected = true;
                    break;
                }
            }
        }
        if (collision_detected) // Collision detected, no need to check other points
        {
            break;
        }
        trajectory.cars_sd.push_back(carsd);
    }
    return collision_detected;
}

bool CollisionChecker::check_limits(Trajectory &trajectory, const double max_speed, const double max_acceleration) {


    char buf[100];

    if (trajectory.max_speed > max_speed )
    {
        sprintf(buf, "Exceeds speed limit, v=%2.2f",  trajectory.max_speed);
        trajectory.limit_message = buf;
        return false;
    }
    if (trajectory.max_total_acceleration > max_acceleration)
    {
        sprintf(buf, "Exceeds acceleration limit, a=%2.2f",  trajectory.max_total_acceleration);
        trajectory.limit_message = buf;
        return false;
    }
    return true;
}

double CollisionChecker::loop_distance(double s1, double s2) {

    const double kLoopLength = 6946;

    if (s1 > kLoopLength && s2 < 500)
    {
        s1 -= kLoopLength;
    }
    else if (s2 > kLoopLength && s1 < 500)
    {
        s2 -= kLoopLength;
    }

    return s1 - s2;
}
