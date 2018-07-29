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

    int current_lane = MapTransformer::d2lane(frenet_d);


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

        PathHelper::estimate_v_a(previous_path_x, previous_path_y, min_tail_points, t_start_speed,
                                 t_start_acceleration);
    }

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

    bool can_left = current_lane > 0;
    bool can_right = current_lane < 2;

    possible_lanes.push_back(current_lane);    // First try to to stay in the current lane. That's most comfortable.

    if (can_left) {
        possible_lanes.push_back(current_lane - 1);
    }
    if (can_right) {
        possible_lanes.push_back(current_lane + 1);
    }

    int lane_index = 0;

    Trajectory best_trajectory;

    while (t_target_speed > kMinSpeed) {

        vector<double> profile = SpeedHelper::calculateAccelerationProfile(
                t_start_speed, t_start_acceleration,
                t_target_speed, time_frame,
                kMaxAcceleration, kMaxJerk);

        double t_end_speed = profile[0];
        double t_distance = profile[1];
        double t_acc = profile[2];


        target_lane = possible_lanes[lane_index];


        Trajectory trajectory = TrajectoryHelper::buildTrajectory(t_start_x, t_start_y, t_start_yaw, t_start_s, profile,
                                                              target_lane,
                                                              time_frame, map_waypoints_x, map_waypoints_y,
                                                              map_waypoints_s);


        unsigned long existing_points = next_x_vals.size();

        bool is_valid = true;

        for (int i = 0; i < trajectory.path_x.size(); i++) {
            double my_yaw = t_start_yaw;
            if (i > 0) {
                double my_yaw = atan2(trajectory.path_y[i] - trajectory.path_y[i - 1], trajectory.path_x[i] - trajectory.path_x[i - 1]);
            }
            double my_x = trajectory.path_x[i];
            double my_y = trajectory.path_y[i];

            vector<double> my_frenet = MapTransformer::getFrenet(my_x, my_y, my_yaw, map_waypoints_x,
                                                                 map_waypoints_y);
            double my_s = my_frenet[0];
            double my_d = my_frenet[1];

            for (int car = 0; car < sensor_fusion.size(); car++) {
                double car_vx = sensor_fusion[car][3];
                double car_vy = sensor_fusion[car][4];
                double car_v = sqrt(car_vx * car_vx + car_vy * car_vy);
                double car_s = sensor_fusion[car][5] + (existing_points + i + 1) * 0.02 * car_v;
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

                if (car_s > my_s && car_s - my_s < min_distance) {
                    if (fabs(my_d - car_d) < 3) {
                        is_valid = false;
                    }
                }
            }
        }


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

        if (!is_valid) {
            if (lane_index < possible_lanes.size() - 1) {
                lane_index++; // try next lane
                cout << "Trying lane " << possible_lanes[lane_index] << endl;
            } else {
                lane_index = 0;
                t_target_speed -= 0.447;    // Try to drive slower
                cout << "Reducing speed to " << t_target_speed << endl;

                if (t_target_speed < kMinSpeed)
                {
                    cout << "Traveling at the minimum speed: " << t_target_speed << endl;
                    best_trajectory = trajectory;
                }

            }
        } else {
            cout << "Trajectory is Ok, target speed: " << t_target_speed << endl;
            best_trajectory = trajectory;
            break;
        }
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



