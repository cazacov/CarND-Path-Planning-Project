//
// Created by victor on 29.07.18.
//

#include "trajectory_helper.h"
#include "speed_helper.h"
#include "maptransformer.h"

#include <math.h>
#include <vector>
#include <iostream>
#include <iomanip>

using namespace std;

Trajectory TrajectoryHelper::buildTrajectory(double start_x, double start_y, double start_yaw, double start_s, double start_d, int start_lane,
                                             AccelerationProfile& profile, int target_lane, double time, bool is_changing_lane) {

    vector<double> ptsx;
    vector<double> ptsy;

    // add two start points to define start and initial start_yaw of the trajectory

    double prev_x = start_x - cos(start_yaw);
    double prev_y = start_y - sin(start_yaw);

    ptsx.push_back(prev_x);
    ptsx.push_back(start_x);

    ptsy.push_back(prev_y);
    ptsy.push_back(start_y);

    double t1;
    double t2;
    double t3;
    double t4;

    double d1 = MapTransformer::lane2d(target_lane);
    double d2 = MapTransformer::lane2d(target_lane);
    double d3 = MapTransformer::lane2d(target_lane);
    double d4 = MapTransformer::lane2d(target_lane);

    if (start_lane == target_lane)
    {

        if (!is_changing_lane) {
            // prefer more precise trajectory
            t1 = 0.3;
            t2 = 0.6;
            t3 = 0.8;
            t4 = 1;
        }
        else {
            // prefer more smooth trajectory
            t1 = 0.3;
            d1 = d1 * 0.8 + start_d * 0.2;
            t2 = 0.5;
            d2 = d2 * 0.9 + start_d * 0.1;
            t3 = 0.8;
            t4 = 1;
        }
    }
    else {
        // prefer more smooth trajectory
        t1 = 0.7;
        t2 = 0.8;
        t3 = 0.9;
        t4 = 1;
    }

    double s0 = profile.get_s(time*t1);
    double s1 = profile.get_s(time*t2);
    double s2 = profile.get_s(time*t3);
    double s3 = profile.get_s(time*t4);

    // add another points
    vector<double> next_wp0 = MapTransformer::getXY(start_s + s0, d1, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = MapTransformer::getXY(start_s + s1, d2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = MapTransformer::getXY(start_s + s2, d3, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp3 = MapTransformer::getXY(start_s + s3, d4, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);
    ptsx.push_back(next_wp3[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);
    ptsy.push_back(next_wp3[1]);

    // Shift car reference to 0 degrees
    for (int i = 0; i < ptsx.size(); i++)
    {
        double shift_x = ptsx[i] - start_x;
        double shift_y = ptsy[i] - start_y;

        ptsx[i] = (shift_x *cos(0 - start_yaw) - shift_y*sin(0-start_yaw));
        ptsy[i] = (shift_x *sin(0 - start_yaw) + shift_y*cos(0-start_yaw));
    }

    Trajectory result;
    result.start_lane = start_lane;
    result.target_lane = target_lane;

    // check points
    for (int i = 0; i < ptsx.size() - 1; i++) {
        if (ptsx[i] >= ptsx[i+1])
        {
            cout << "OMG";
        }
    }

    result.spline.set_points(ptsx, ptsy);

    generatePath(start_x, start_y, start_yaw, profile, time, result);

    return result;
}


void TrajectoryHelper::generatePath(double start_x, double start_y, double start_yaw, AccelerationProfile &profile,
                                    double time, Trajectory& trajectory) {
    trajectory.path_x.clear();
    trajectory.path_y.clear();
    trajectory.path_v.clear();
    trajectory.path_a.clear();

    double t = 0;
    double dt = 0.02;

    for (int i = 0; i < time / 0.02; i++)
    {
        t += dt;

        double x_point = profile.get_s(t);
        double y_point = trajectory.spline(x_point);

        double x_ref = x_point;
        double y_ref = y_point;

        double deriv1 = trajectory.spline.deriv(1, x_point);
        double deriv2 = trajectory.spline.deriv(2, x_point);

        double local_yaw = atan2(deriv1, 1);
        dt = 0.02 * cos(local_yaw);

        // curvature
        double k = fabs(deriv2) / pow(1 + deriv1 * deriv1, 3.0/2.0 );

        x_point = x_ref * cos(start_yaw) - y_ref*sin(start_yaw);
        y_point = x_ref * sin(start_yaw) + y_ref*cos(start_yaw);

        x_point += start_x;
        y_point += start_y;

        trajectory.path_x.push_back(x_point);
        trajectory.path_y.push_back(y_point);
        trajectory.path_v.push_back(profile.get_v(t));
        trajectory.path_a.push_back(profile.get_a(t));
        trajectory.path_k.push_back(k);
    }
    trajectory.update_metrics(0.02);
}

bool TrajectoryHelper::check_collision(const vector<vector<double>> &sensor_fusion, double t_start_yaw,
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
        if (i > 0) {
            my_yaw = atan2(trajectory.path_y[i] - trajectory.path_y[i - 1],
                                  trajectory.path_x[i] - trajectory.path_x[i - 1]);
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

            // TODO: check end of highway loop
            if (car_s0 + 10 < my_s && car_lane == trajectory.start_lane)
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

            if (car_s > (my_s - min_back_s) && (car_s - my_s) < min_s_distance) {
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

bool TrajectoryHelper::check_limits(Trajectory &trajectory, const double max_speed, const double max_acceleration) {
    if (trajectory.max_speed > max_speed )
    {
        cout << "\tExceeds speed limit " << std::setw(2) << trajectory.max_speed;
        return false;
    }
    if (trajectory.max_acceleration > max_acceleration)
    {
        cout << "\tExceeds acceleration limit " << std::setw(2) << trajectory.max_acceleration;
        return false;
    }
    return true;
}

