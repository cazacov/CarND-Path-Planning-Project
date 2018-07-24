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
const double kPlanningTime  =   3;  // seconds
const double kMaxSpeed = 49 * 0.447; // 49 MPH

int iteration = 0;
double last_acc = 0;
double last_v = 0;


void
PathPlanner::planPath(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
                      vector<vector<double>> sensor_fusion,
                      vector<double> previous_path_x, vector<double> previous_path_y,
                      double end_path_s, double end_path_d) {



    vector<double> frenet = MapTransformer::getFrenet(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);

    double frenet_s = frenet[0];
    double frenet_d = frenet[1];

    int target_lane = 1;

    // Starting point of new trajectory
    double t_start_x = car_x;
    double t_start_y = car_y;
    double t_start_yaw = car_yaw;
    double t_start_speed = car_speed;
    double t_start_acceleration = 0;
    double t_start_s = car_s;


    if (previous_path_x.size() > 40)
    {
        previous_path_x.resize(40);
        previous_path_y.resize(40);
    }

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
        t_start_yaw = atan2(y_back - y_prev, x_back - x_prev);

        vector<double> end_frenet = MapTransformer::getFrenet(t_start_x, t_start_y, t_start_yaw, map_waypoints_x, map_waypoints_y);
        t_start_s = end_frenet[0];

        // Use polynomial fitting to estimate speed and acceleration at last min_tail_points of the previous path
        vector<double> poly_x;
        vector<double>  poly_y;
        int start_idx = previous_path_x.size() - min_tail_points;
        for (int i = start_idx; i <  previous_path_x.size(); i++)
        {
            poly_x.push_back(i-start_idx);
            poly_y.push_back(MapTransformer::distance(previous_path_x[start_idx], previous_path_y[start_idx],previous_path_x[i], previous_path_y[i])+5);
        }

        // Use polynomial approximation to estimate speed and acceleration at the end of the previous path
        vector<double> coeff;
        MathHelper::polyfit(poly_x, poly_y, coeff, 2);

        double v0 = coeff[1] / 0.02;
        double a0 = coeff[2] / (0.02 * 0.02) * 2;

        double t = 0.02 * (min_tail_points - 1);

        t_start_acceleration = a0;
        t_start_speed =  v0 +  a0 * t;
    }

    next_x_vals.clear();
    next_y_vals.clear();
    for (int i = 0; i < previous_path_x.size(); i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    double start_time = previous_path_x.size() * 0.02;  // we already have path points for start_time seconds

    // estimate max speed that can be achieved in planning_time
    double time_frame = kPlanningTime - start_time;

    double t_target_speed = kMaxSpeed;

    vector<double> profile = SpeedHelper::calculateAccelerationProfile(
            t_start_speed, t_start_acceleration,
            t_target_speed, time_frame,
            kMaxAcceleration, kMaxJerk);

    double t_end_speed = profile[0];
    double t_distance = profile[1];
    double t_acc = profile[2];


    /*1
    vector<double> jmt = SpeedHelper::solveJmt(t_start_speed, t_end_speed, t_start_acceleration, time_frame,
                                               t_distance);
    */

    tk::spline spline = buildTrajectory(t_start_x, t_start_y, t_start_yaw, t_start_s, profile, target_lane, time_frame);

    vector<double> new_path_x;
    vector<double> new_path_y;
    vector<double> new_path_v;
    vector<double> new_path_a;

    generatePath(t_start_x, t_start_y, t_start_yaw, profile, spline, next_x_vals.size(), new_path_x, new_path_y,
                 new_path_v, new_path_a);

    for (int i = 0; i < new_path_x.size(); i++) {
        next_x_vals.push_back(new_path_x[i]);
        next_y_vals.push_back(new_path_y[i]);
    }

    vector<double> dx;
    for (int i = 0; i < next_x_vals.size()-1; i++) {
        dx.push_back((next_x_vals[i+1] - next_x_vals[i]) / 0.02);
    }

    cout << setw(5) << iteration
         << "\t"
         << "car_speed=" << fixed << setw(6) << setprecision(3)  << car_speed * 0.447
         << "\t"
         << "t_speed=" << fixed << setw(6) << setprecision(3)  << t_start_speed
         << "\t"
         << "t_acc=" << fixed << setw(6) << setprecision(3) << t_start_acceleration
         << "\t"
         << "l_v=" << fixed << setw(6) << setprecision(3) << last_v
         << "\t"
         << "l_acc=" << fixed << setw(6) << setprecision(3) << last_acc
         << "\t"
         << "end_speed=" << fixed << setw(6) << setprecision(3)  << t_end_speed
         << "\t"
         << "dist=" << fixed << setw(6) << setprecision(3)  << t_distance
         << "\t"
         << "ta=" << fixed << setw(6) << setprecision(3)  << t_acc
         << "\t"
         << "time=" << fixed << setw(6) << setprecision(3)  << time_frame

         << "\t"
         << "start_s=" << setw(6) << t_start_s
         << endl;

    last_v = new_path_v.back();
    last_acc = new_path_a.back();

    iteration++;

}

tk::spline PathPlanner::buildTrajectory(double x, double y, double yaw, double s, vector<double> profile, int lane, double time) {
    vector<double> ptsx;
    vector<double> ptsy;

    // add two start points to define start and initial yaw of the trajectory

    double prev_x = x - cos(yaw);
    double prev_y = y - sin(yaw);

    ptsx.push_back(prev_x);
    ptsx.push_back(x);

    ptsy.push_back(prev_y);
    ptsy.push_back(y);

    double s0 = SpeedHelper::applyProfile(profile, time*0.5)[0];
    double s1 = SpeedHelper::applyProfile(profile, time*0.75)[0];
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
        double shift_x = ptsx[i] - x;
        double shift_y = ptsy[i] - y;

        ptsx[i] = (shift_x *cos(0 - yaw) - shift_y*sin(0-yaw));
        ptsy[i] = (shift_x *sin(0 - yaw) + shift_y*cos(0-yaw));
    }

    tk::spline spln;
    spln.set_points(ptsx, ptsy);

    return spln;
}

double PathPlanner::applyJmt(const vector<double>& coefficients, double time)
{
    double t2 = time*time;
    double t3 = t2*time;
    double t4 = t3*time;
    double t5 = t4*time;

    return coefficients[0] + coefficients[1] * time + coefficients[2] * t2
           + coefficients[3] * t3 + coefficients[4] * t4 + coefficients[5] * t5;
}

void PathPlanner::generatePath(double x, double y, double yaw, vector<double> profile, tk::spline trajectory,
                               unsigned long existing_points, vector<double>& path_x, vector<double>& path_y, vector<double>& path_v, vector<double>& path_a) {

    path_x.clear();
    path_y.clear();

    for (int i = 1; i <= 50 - existing_points; i++)
    {
        vector<double> data = SpeedHelper::applyProfile(profile, i * 0.02);

        double x_point = data[0];
        double y_point = trajectory(x_point);

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = x_ref * cos(yaw) - y_ref*sin(yaw);
        y_point = x_ref * sin(yaw) + y_ref*cos(yaw);

        x_point += x;
        y_point += y;

        path_x.push_back(x_point);
        path_y.push_back(y_point);
        path_v.push_back(data[1]);
        path_a.push_back(data[2]);
    }
}

