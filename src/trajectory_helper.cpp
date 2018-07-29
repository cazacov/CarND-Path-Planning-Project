//
// Created by victor on 29.07.18.
//

#include "trajectory_helper.h"
#include "speed_helper.h"
#include "maptransformer.h"

#include <math.h>
#include <vector>

using namespace std;

tk::spline TrajectoryHelper::buildTrajectory(
        double start_x, double start_y, double start_yaw, double s,
        const vector<double> profile, int lane, double time,
        const std::vector<double>& map_waypoints_x,
        const std::vector<double>& map_waypoints_y,
        const std::vector<double>& map_waypoints_s) {

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
    double s1 = SpeedHelper::applyProfile(profile, time*0.8)[0];
    double s2 = SpeedHelper::applyProfile(profile, time)[0];

    // add another points
    vector<double> next_wp0 = MapTransformer::getXY(s + s0, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = MapTransformer::getXY(s + s1, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = MapTransformer::getXY(s + s2, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    // Shift car reference to 0 degrees
    for (int i = 0; i < ptsx.size(); i++)
    {
        double shift_x = ptsx[i] - start_x;
        double shift_y = ptsy[i] - start_y;

        ptsx[i] = (shift_x *cos(0 - start_yaw) - shift_y*sin(0-start_yaw));
        ptsy[i] = (shift_x *sin(0 - start_yaw) + shift_y*cos(0-start_yaw));
    }

    tk::spline spln;
    spln.set_points(ptsx, ptsy);

    return spln;
}

void TrajectoryHelper::generatePath(double start_x, double start_y, double start_yaw, vector<double> profile,
                               tk::spline trajectory, double time,
                               std::vector<double>& path_x, std::vector<double>& path_y, std::vector<double>& path_v, std::vector<double>& path_a) {

    path_x.clear();
    path_y.clear();

    for (int i = 1; i <= time / 0.02; i++)
    {
        vector<double> data = SpeedHelper::applyProfile(profile, i * 0.02);

        double x_point = data[0];
        double y_point = trajectory(x_point);

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = x_ref * cos(start_yaw) - y_ref*sin(start_yaw);
        y_point = x_ref * sin(start_yaw) + y_ref*cos(start_yaw);

        x_point += start_x;
        y_point += start_y;

        path_x.push_back(x_point);
        path_y.push_back(y_point);
        path_v.push_back(data[1]);
        path_a.push_back(data[2]);
    }
}
