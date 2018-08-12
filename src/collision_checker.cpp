#include <math.h>

#include "collision_checker.h"
#include "maptransformer.h"

#include <iostream>
#include <iomanip>


using namespace std;

bool CollisionChecker::check_collision(const vector<vector<double>> &sensor_fusion, double t_start_yaw,
                                       int current_lane,
                                       Trajectory &trajectory, double start_time) {
    bool result = true;

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

            // TODO: check end of highway loop
            if (loop_distance(my_s, car_s0) > 10 && car_lane == trajectory.start_lane)
            {
                // Ignore cars behind me. They should keep safe distance.
                continue;
            }


            double car_yaw = atan2(car_vy, car_vx);
            vector<double> car_xy = MapTransformer::getXY(car_s, car_d, map_waypoints_s, map_waypoints_x,
                                                          map_waypoints_y);



            bool trajectory_changes_lane = trajectory.start_lane != trajectory.target_lane;

            // TODO: check end of highway loop
            double min_s_distance = trajectory.path_v[i] * 1.5;   // At least 1.5 second distance to next car
            double min_back_s = 0;   // Ignore cars behind us
            double min_d_distance = 2.0;

            if (trajectory_changes_lane)
            {
                min_d_distance = 2.6;   // more defensive collision check

                if (car_lane == trajectory.start_lane)
                {
                    min_s_distance = trajectory.path_v[i] * 1; // We can shortly pass near another car in my lane when changing to another lane
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
                    trajectory.collision_other_id = car_id;
                    trajectory.collision_time = time;
                    trajectory.collision_min_d = min_d_distance;
                    trajectory.collision_min_s = min_s_distance;
                    result = false;
                    break;
                }
            }
        }
        if (!result)
        {
            break;
        }
        trajectory.cars_sd.push_back(carsd);
    }
    return result;
}

bool CollisionChecker::check_limits(Trajectory &trajectory, const double max_speed, const double max_acceleration) {
    if (trajectory.max_speed > max_speed )
    {
        cout << "\tExceeds speed limit " << std::setw(2) << trajectory.max_speed;
        return false;
    }
    if (trajectory.max_total_acceleration > max_acceleration)
    {
        cout << "\tExceeds acceleration limit " << std::setw(3) << trajectory.max_total_acceleration;
        return false;
    }
    return true;
}

double CollisionChecker::loop_distance(double s1, double s2) {
    return s1 - s2;
}
