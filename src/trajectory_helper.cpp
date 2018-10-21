#include "trajectory_helper.h"
#include "speed_helper.h"
#include "maptransformer.h"

#include <math.h>
#include <vector>
#include <iostream>
#include <iomanip>

using namespace std;

Trajectory TrajectoryHelper::buildTrajectory(double start_x, double start_y, double start_yaw, double start_s, double start_d, int start_lane,
    double start_speed, AccelerationProfile& profile, int target_lane, double time, bool is_changing_lane) {

    vector<double> ptsx;
    vector<double> ptsy;

    // add two start points to define start position and initial start_yaw of the trajectory

    double prev_x = start_x - cos(start_yaw);
    double prev_y = start_y - sin(start_yaw);

    ptsx.push_back(prev_x);
    ptsx.push_back(start_x);

    ptsy.push_back(prev_y);
    ptsy.push_back(start_y);

    // Key points for spline generation

    // times
    double t1;
    double t2;
    double t3;
    double t4;

    // d-coordinates
    double d1 = MapTransformer::lane2d(target_lane);
    double d2 = MapTransformer::lane2d(target_lane);
    double d3 = MapTransformer::lane2d(target_lane);
    double d4 = MapTransformer::lane2d(target_lane);

    if (start_lane == target_lane)
    {

        if (!is_changing_lane) {
            // Keep current lane. Prefer more precise trajectory
            t1 = 0.3;
            t2 = 0.6;
            t3 = 0.8;
            t4 = 1;
        }
        else {
            // finish lane changing maneuver. Prefer more smooth trajectory
            t1 = 0.3;
            d1 = d1 * 0.8 + start_d * 0.2;
            t2 = 0.5;
            d2 = d2 * 0.9 + start_d * 0.1;
            t3 = 0.8;
            t4 = 1;
        }
    }
    else {
        // changing the lane. Prefer more smooth trajectory
        t1 = 0.7;
        t2 = 0.8;
        t3 = 0.9;
        t4 = 1;
    }


    double s0 = profile.get_s(time*t1);
    double s1 = profile.get_s(time*t2);
    double s2 = profile.get_s(time*t3);
    double s3 = profile.get_s(time*t4);

    // add key points
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

    // Call spline interpolation library
    result.spline.set_points(ptsx, ptsy);

    // Generate waypoints
    generatePath(start_x, start_y, start_yaw, profile, time, result);

    // Calculate maximum speed and acceleration
    result.update_metrics(0.02, start_speed, start_x, start_y);
    return result;
}


void TrajectoryHelper::generatePath(double start_x, double start_y, double start_yaw, AccelerationProfile &profile,
                                    double time, Trajectory& trajectory) {
    trajectory.path_x.clear();
    trajectory.path_y.clear();
    trajectory.path_v.clear();

    double t = 0;
    double dt = 0.02;

    for (int i = 0; i < time / 0.02; i++)
    {
        t += dt;

        double x_point = profile.get_s(t);
        double y_point = trajectory.spline(x_point);

        double x_ref = x_point;
        double y_ref = y_point;

        // transform back to world coordinate system
        x_point = x_ref * cos(start_yaw) - y_ref*sin(start_yaw);
        y_point = x_ref * sin(start_yaw) + y_ref*cos(start_yaw);

        x_point += start_x;
        y_point += start_y;

        trajectory.path_x.push_back(x_point);
        trajectory.path_y.push_back(y_point);
        trajectory.path_v.push_back(profile.get_v(t));


        // Calculate curvature

        double deriv1 = trajectory.spline.deriv(1, x_point);
        double deriv2 = trajectory.spline.deriv(2, x_point);
        dt = 0.02 / sqrt(1 + deriv1*deriv1);
        double k = fabs(deriv2) / pow(1 + deriv1 * deriv1, 3.0/2.0 );

        trajectory.path_k.push_back(k);
    }
}



