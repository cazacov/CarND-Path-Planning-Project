#include <math.h>
#include <iostream>
#include <iomanip>
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

#include "pathplanner.h"
#include "maptransformer.h"
#include "math_helper.h"
#include "speed_helper.h"
#include "path_helper.h"
#include "trajectory_helper.h"

PathPlanner::PathPlanner(const std::vector<double> &_map_waypoints_x,
                         const std::vector<double> &_map_waypoints_y,
                         const std::vector<double> &_map_waypoints_s,
                         const std::vector<double> &_map_waypoints_dx,
                         const std::vector<double> &_map_waypoints_dy)
        : map_waypoints_x(_map_waypoints_x),
          map_waypoints_y(_map_waypoints_y),
          map_waypoints_s(_map_waypoints_s),
          map_waypoints_dx(_map_waypoints_dx),
          map_waypoints_dy(_map_waypoints_dy) {

}

const double kMaxAcceleration = 5;  // m/s^2
const double kMaxJerk = 5;  // m/s^3
const double kPlanningTime = 3;  // seconds
const double kMaxSpeed = 48 * 0.447; // 48 MPH
const double kMinSpeed = 20 * 0.447; // 20 MPH  if cannot avoid collision brake to some minimum speed

int iteration = 0;
double last_acc = 0;
double last_v = 0;
int lane_change_lock = 0;
int fix_lane = 0;



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
    double t_start_d = car_d;

    if (previous_path_x.size() > 25) {
        previous_path_x.resize(25);
        previous_path_y.resize(25);
    }

    const int min_tail_points = 10;
    if (previous_path_x.size() >= min_tail_points) // we have some data from previous trajectory
    {
        PathHelper::estimate_x_y_yaw(previous_path_x, previous_path_y, min_tail_points, t_start_x, t_start_y,
                                     t_start_yaw);

        vector<double> end_frenet = MapTransformer::getFrenet(t_start_x, t_start_y, t_start_yaw, map_waypoints_x,
                                                              map_waypoints_y);
        t_start_s = end_frenet[0];
        t_start_d = end_frenet[1];

        PathHelper::estimate_v_a(previous_path_x, previous_path_y, min_tail_points, t_start_speed,
                                 t_start_acceleration);
    }

    int start_lane = MapTransformer::d2lane(t_start_d);

    next_x_vals.clear();
    next_y_vals.clear();
    for (int i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    double start_time = previous_path_x.size() * 0.02;  // we already have path points for start_time seconds

    // estimate max speed that can be achieved in planning_time
    double time_frame = kPlanningTime - start_time;

    double t_target_speed = kMaxSpeed;

    vector<int> possible_lanes;

    bool can_left = start_lane > 0;
    bool can_right = start_lane < 2;

    if (lane_change_lock == 1)
    {
        cout << "Lane free";
    }
    if (lane_change_lock > 0)
    {
        lane_change_lock--;
    }

    if (lane_change_lock > 0 )
    {
        possible_lanes.push_back(fix_lane);
    }
    else {
        possible_lanes.push_back(start_lane);    // First try to to stay in the current lane. That's most comfortable.
        if (can_left) {
            possible_lanes.push_back(start_lane - 1);
        }
        if (can_right) {
            possible_lanes.push_back(start_lane + 1);
        }
    }

    char buf[4];
    for (int i = 0; i < possible_lanes.size(); i++)
    {
        buf[i] = possible_lanes[i] + '0';
    }
    buf[possible_lanes.size()] = '\0';


    int lane_index = 0;

    TrajectoryHelper trajectoryHelper(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx,
                                      map_waypoints_dy);

    Trajectory best_trajectory;

    cout << setw(5) << iteration << setw(4) << left << buf;

    while (t_target_speed > kMinSpeed) {

        vector<double> profile = SpeedHelper::calculateAccelerationProfile(
                t_start_speed, t_start_acceleration,
                t_target_speed, time_frame,
                kMaxAcceleration, kMaxJerk);

        double t_end_speed = profile[0];
        double t_distance = profile[1];
        double t_acc = profile[2];


        target_lane = possible_lanes[lane_index];


        Trajectory trajectory = trajectoryHelper.buildTrajectory(
                t_start_x, t_start_y, t_start_yaw, t_start_s, start_lane,
                profile, target_lane, time_frame);

        cout << "\tL: "<< target_lane << "\tv:" << setw(6) << t_target_speed;


        bool is_valid = trajectoryHelper.validate(sensor_fusion, t_start_yaw, start_lane, trajectory, start_time);

/*
        cout << setw(5) << iteration
             << "\t"
             << "car_speed=" << fixed << setw(6) << setprecision(3) << car_speed * 0.447
             << "\t"
             << "t_speed=" << fixed << setw(6) << setprecision(3) << t_start_speed
             << "\t"
             << "t_acc=" << fixed << setw(6) << setprecision(3) << t_start_acceleration
             << "\t"
             << "l_v=" << fixed << setw(6) << setprecision(3) << last_v
             << "\t"
             << "l_acc=" << fixed << setw(6) << setprecision(3) << last_acc
             << "\t"
             << "end_speed=" << fixed << setw(6) << setprecision(3) << t_end_speed
             << "\t"
             << "dist=" << fixed << setw(6) << setprecision(3) << t_distance
             << "\t"
             << "ta=" << fixed << setw(6) << setprecision(3) << t_acc
             << "\t"
             << "time=" << fixed << setw(6) << setprecision(3) << time_frame

             << "\t"
             << "start_s=" << setw(6) << t_start_s
             << endl;
*/

        if (!is_valid) {
            cout << "\tHit car " << setw(2) << trajectory.collision_other_id
                << "\ts=" << setw(6) << trajectory.collision_other_s
                << "\td=" << setw(6) << trajectory.collision_other_d
                << "\tmy_s=" << setw(6) << trajectory.collision_my_s
                << "\tmy_d=" << setw(6) << trajectory.collision_my_d
                << "\ttime=" << setw(6) << trajectory.collision_time;
            if (lane_index < possible_lanes.size() - 1) {
                lane_index++; // try next lane
//                cout << "Trying lane " << possible_lanes[lane_index] << endl;
            } else {
                lane_index = 0;
                t_target_speed -= 0.447;    // Try to drive slower

                if (t_target_speed < kMinSpeed) {
                    // Stay in the current lane
                    best_trajectory = trajectoryHelper.buildTrajectory(
                            t_start_x, t_start_y, t_start_yaw, t_start_s, start_lane,
                            profile, start_lane, time_frame);
                }
            }
        } else {
            best_trajectory = trajectory;

            if (start_lane != target_lane && lane_change_lock == 0)
            {
                lane_change_lock = 100;
                fix_lane = target_lane;
                cout << "Lane lock";
            }

            cout << "\tOk" << endl;
            break;
        }
        cout << endl;
    }

    for (int i = 0; i < best_trajectory.path_x.size(); i++) {
        next_x_vals.push_back(best_trajectory.path_x[i]);
        next_y_vals.push_back(best_trajectory.path_y[i]);
    }


    vector<double> dx;
    for (int i = 0; i < next_x_vals.size() - 1; i++) {
        dx.push_back((next_x_vals[i + 1] - next_x_vals[i]) / 0.02);
    }

    /*
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
    */

    //last_v = new_path_v.back();
    //last_acc = new_path_a.back();

    iteration++;

}




double PathPlanner::applyJmt(const vector<double> &coefficients, double time) {
    double t2 = time * time;
    double t3 = t2 * time;
    double t4 = t3 * time;
    double t5 = t4 * time;

    return coefficients[0] + coefficients[1] * time + coefficients[2] * t2
           + coefficients[3] * t3 + coefficients[4] * t4 + coefficients[5] * t5;
}



