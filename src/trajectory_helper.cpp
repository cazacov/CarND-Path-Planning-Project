//
// Created by victor on 29.07.18.
//

#include "trajectory_helper.h"
#include "speed_helper.h"
#include "maptransformer.h"

#include <math.h>
#include <vector>

using namespace std;

Trajectory TrajectoryHelper::buildTrajectory(
        double start_x, double start_y, double start_yaw, double s,
        const vector<double>& profile, int lane, double time) {

    vector<double> ptsx;
    vector<double> ptsy;

    // add two start points to define start and initial start_yaw of the trajectory

    double prev_x = start_x - cos(start_yaw);
    double prev_y = start_y - sin(start_yaw);

    ptsx.push_back(prev_x);
    ptsx.push_back(start_x);

    ptsy.push_back(prev_y);
    ptsy.push_back(start_y);

    double s0 = SpeedHelper::applyProfile(profile, time*0.6)[0];
    double s1 = SpeedHelper::applyProfile(profile, time*0.75)[0];
    double s2 = SpeedHelper::applyProfile(profile, time*0.95)[0];
    double s3 = SpeedHelper::applyProfile(profile, time*1.0)[0];

    // add another points
    vector<double> next_wp0 = MapTransformer::getXY(s + s0, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = MapTransformer::getXY(s + s1, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = MapTransformer::getXY(s + s2, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp3 = MapTransformer::getXY(s + s3, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

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
