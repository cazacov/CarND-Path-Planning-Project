#include <math.h>
#include <iostream>
#include <iomanip>
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

#include "pathplanner.h"
#include "maptransformer.h"
#include "math_helper.h"
#include "speed_helper.h"

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

const double kMaxAcceleration = 5;  // m/s^2
const double kMaxJerk =         5;  // m/s^3
const double kPlanningTime  =   4;  // seconds
const double kMaxSpeed = 49.5 * 0.447; // 49.5 MPH

void
PathPlanner::planPath(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
                      vector<vector<double>> sensor_fusion,
                      vector<double> previous_path_x, vector<double> previous_path_y,
                      double end_path_s, double end_path_d) {



    vector<double> frenet = MapTransformer::getFrenet(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);

    double s = frenet[0];
    double d = frenet[1];



    int target_lane = 1;

    // Starting point of new trajectory
    double t_start_x = car_x;
    double t_start_y = car_y;
    double t_start_yaw = car_yaw;
    double t_start_speed = car_speed;
    double t_start_acceleration = 0;
    double t_start_s = car_s;

    const int min_tail_points = 10;
    if (previous_path_x.size() >= min_tail_points) // we have some data from previous trajectory
    {
        // estimate car speed and yaw
        double x_back = previous_path_x.back();
        double y_back = previous_path_y.back();
        double x_prev = *(previous_path_x.end() - min_tail_points);  // -1 because the end() is referring past-the-end element
        double y_prev = *(previous_path_y.end() - min_tail_points);

        t_start_x = x_back;
        t_start_y = y_back;
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

        t_start_speed = coeff[1] / 0.02;
        t_start_acceleration = coeff[2] / (0.02 * 0.02);

        vector<double> frenet = MapTransformer::getFrenet(t_start_x, t_start_y, t_start_yaw, map_waypoints_x, map_waypoints_y );
    }

    double start_time = previous_path_x.size() * 0.02;  // we already have path points for start_time seconds

    // estimate max speed that can be achieved in planning_time
    double time_frame = kPlanningTime - start_time;

    double t_target_speed = kMaxSpeed;

    double t_end_speed = SpeedHelper::calculateMaxSpeed(t_start_speed, t_start_acceleration, time_frame,
                                                        kMaxAcceleration, t_target_speed);

    cout    << "s=" << setw(6) << s
            << "\t"
            << "d=" << setw(6) << d
            << "\t"
            << "speed=" << setw(6) << t_start_speed
            << "\t"
            << "accel=" << setw(6) << t_start_acceleration
            << "\t"
            << "time_frame=" << setw(6) << time_frame
            << "\t"
            << "t_end_speed=" << setw(6) << t_end_speed
            << endl;

    if (t_end_speed > kMaxSpeed) {
        t_end_speed = kMaxSpeed;
    };

    vector<double> jmt = SpeedHelper::solveJmt(car_s, t_start_speed, t_end_speed, t_start_acceleration, time_frame);

    tk::spline spline = buildTrajectory(t_start_x, t_start_y, t_start_yaw, car_s, jmt, target_lane, time_frame);

    next_x_vals.clear();
    next_y_vals.clear();
    for (int i = 0; i < previous_path_x.size(); i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    double target_x = 30.0;
    double target_y = spline(target_x);
    double target_distance = MapTransformer::distance(0,0,target_x, target_y);

    //generate_trajectory(t_start_x, t_start_y, t_start_yaw, t_start_speed2, t_start_acceleration, target_lane )


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

tk::spline PathPlanner::buildTrajectory(double x, double y, double yaw, double s, vector<double> jmt, int lane, double time) {
    vector<double> ptsx;
    vector<double> ptsy;

    // add two start points to define start and initial yaw of the trajectory

    double prev_x = x - cos(yaw);
    double prev_y = y - cos(yaw);

    ptsx.push_back(prev_x);
    ptsx.push_back(x);

    ptsy.push_back(prev_y);
    ptsy.push_back(y);

    // add another points
    vector<double> next_wp0 = MapTransformer::getXY(applyJmt(jmt, time*0.75), 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = MapTransformer::getXY(applyJmt(jmt, time*0.8), 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = MapTransformer::getXY(applyJmt(jmt, time), 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    // Shift car reference to 0 degrees
    for (int i = 0; i < ptsx.size(); i++)
    {
        double shift_x = ptsx[i] - x;
        double shift_y = ptsy[i] - y;

        ptsx[i] = (shift_x *cos(0 - yaw) - shift_y*sin(0-yaw));
        ptsy[i] = (shift_x *sin(0 - yaw) + shift_y*cos(0-yaw));
    }

    tk::spline spln;
    spln.set_points(ptsx, ptsy);

    return spln;
}

double PathPlanner::applyJmt(vector<double> coefficients, double time)
{
    double t2 = time*time;
    double t3 = t2*time;
    double t4 = t3*time;
    double t5 = t4*time;

    return coefficients[0] + coefficients[1] * time + coefficients[2] * t2
           + coefficients[3] * t3 + coefficients[4] * t4 + coefficients[5] * t5;
}

