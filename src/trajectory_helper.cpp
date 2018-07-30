//
// Created by victor on 29.07.18.
//

#include "trajectory_helper.h"
#include "speed_helper.h"
#include "maptransformer.h"

#include <math.h>
#include <vector>
#include <iostream>

using namespace std;

Trajectory TrajectoryHelper::buildTrajectory(
        double start_x, double start_y, double start_yaw, double s,
        int start_lane, const vector<double>& profile, int target_lane, double time) {

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

    if (start_lane == target_lane)
    {
        // prefer more precise trajectory
        t1 = 0.4;
        t2 = 0.7;
        t3 = 0.9;
        t4 = 1;
    }
    else {
        // prefer more smooth trajectory
        t1 = 0.7;
        t2 = 0.8;
        t3 = 0.9;
        t4 = 1;
    }

    double s0 = SpeedHelper::applyProfile(profile, time*t1)[0];
    double s1 = SpeedHelper::applyProfile(profile, time*t2)[0];
    double s2 = SpeedHelper::applyProfile(profile, time*t3)[0];
    double s3 = SpeedHelper::applyProfile(profile, time*t4)[0];

    // add another points
    vector<double> next_wp0 = MapTransformer::getXY(s + s0, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = MapTransformer::getXY(s + s1, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = MapTransformer::getXY(s + s2, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp3 = MapTransformer::getXY(s + s3, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

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


void TrajectoryHelper::generatePath(double start_x, double start_y, double start_yaw, const std::vector<double> &profile,
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

        vector<double> data = SpeedHelper::applyProfile(profile, t);

        double x_point = data[0];
        double y_point = trajectory.spline(x_point);

        double x_ref = x_point;
        double y_ref = y_point;

        double local_yaw = atan2(trajectory.spline.deriv(1, x_point), 1);
        dt = 0.02 * cos(local_yaw);

        x_point = x_ref * cos(start_yaw) - y_ref*sin(start_yaw);
        y_point = x_ref * sin(start_yaw) + y_ref*cos(start_yaw);

        x_point += start_x;
        y_point += start_y;

        trajectory.path_x.push_back(x_point);
        trajectory.path_y.push_back(y_point);
        trajectory.path_v.push_back(data[1]);
        trajectory.path_a.push_back(data[2]);
    }
}

bool TrajectoryHelper::validate(const vector<vector<double>> &sensor_fusion, double t_start_yaw, int current_lane,
                           Trajectory &trajectory, double start_time) {
    bool result = true;

    for (int i = 0; i < trajectory.path_x.size(); i++) {

        double time = start_time + (i + 1) * 0.02;
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

        for (int car = 0; car < sensor_fusion.size(); car++) {
            int car_id  = sensor_fusion[car][0];
            double car_x = sensor_fusion[car][1];
            double car_y = sensor_fusion[car][2];
            double car_vx = sensor_fusion[car][3];
            double car_vy = sensor_fusion[car][4];
            double car_v = sqrt(car_vx * car_vx + car_vy * car_vy);
            double car_s = sensor_fusion[car][5] + time * car_v;
            double car_d = sensor_fusion[car][6];
            int car_lane = MapTransformer::d2lane(car_d);
            double car_yaw = atan2(car_vy, car_vx);
            vector<double> car_xy = MapTransformer::getXY(car_s, car_d, map_waypoints_s, map_waypoints_x,
                                                          map_waypoints_y);

            double min_distance = 30;
            if (car_lane != current_lane)   // add penalty for changing lane
            {
                min_distance += 10;
            }

            if (car_s > (my_s - 10) && car_s - my_s < min_distance) {
                if (fabs(my_d - car_d) < 2.5) {

                    trajectory.collision_my_s = my_s;
                    trajectory.collision_my_d = my_d;
                    trajectory.collision_other_s = car_s;
                    trajectory.collision_other_d = car_d;
                    trajectory.collision_other_id = car_id;
                    trajectory.collision_time = time;
                    result = false;
                    break;
                }
            }
        }
        if (!result)
        {
            break;
        }
    }
    return result;
}