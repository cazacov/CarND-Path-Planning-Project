#include <math.h>
#include <iostream>
#include <iomanip>
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

#include "pathplanner.h"
#include "maptransformer.h"
#include "math_helper.h"
#include "speed_helper.h"
#include "trajectory_helper.h"
#include "collision_checker.h"
#include "log_helper.h"

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

const double kMaxTanAcceleration = 5;  // m/s^2
const double kMaxTotalAcceleration = 8;  // m/s^2
const double kMaxJerk = 5;  // m/s^3
const double kPlanningTime = 3;  // Plan trajectory for next 3 seconds
const double kMaxSpeed = 48 * 0.447; // 48 MPH
const double kMinSpeed = 20 * 0.447; // 20 MPH  if cannot avoid collision brake to some minimum speed
const int    kReuseNPoints = 12;

int iteration = 0;
int lane_change_lock = 0;
int fix_lane = 0;

void
PathPlanner::planPath(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
                      vector<vector<double>> sensor_fusion,
                      vector<double> previous_path_x, vector<double> previous_path_y,
                      double end_path_s, double end_path_d) {




    // Starting point of a new trajectory
    double t_start_x = car_x;
    double t_start_y = car_y;
    double t_start_yaw = car_yaw;
    double t_start_speed = car_speed;
    double t_start_acceleration = 0;
    double t_start_s = car_s;
    double t_start_d = car_d;

    // Re-use only first kReuseNPoints points of the previous path
    if (previous_path_x.size() > kReuseNPoints)
    {
        previous_path_x.resize(kReuseNPoints);
        previous_path_y.resize(kReuseNPoints);
    }

    // Calculate initial speed and coordinates
    const int min_tail_points = 10;
    if (previous_path_x.size() >= min_tail_points) // we have some data from previous trajectory
    {
        SpeedHelper::estimate_x_y_yaw(previous_path_x, previous_path_y, 2, t_start_x, t_start_y,
                                     t_start_yaw);

        vector<double> end_frenet = MapTransformer::getFrenet(t_start_x, t_start_y, t_start_yaw, map_waypoints_x,
                                                              map_waypoints_y);
        t_start_s = end_frenet[0];
        t_start_d = end_frenet[1];

        SpeedHelper::estimate_v_a(previous_path_x, previous_path_y, min_tail_points, t_start_speed,
                                 t_start_acceleration);
    }

    int start_lane = MapTransformer::d2lane(t_start_d);

    // Carry first kReuseNPoints from the previous path to the new one
    next_x_vals.clear();
    next_y_vals.clear();
    for (int i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // Make array of lanes that we are going to consider in trajectory planning
    vector<int> possible_lanes;

    // Check if are right now doing lane changing maneuver
    bool is_changing_lane = lane_change_lock > 0;

    // Decrement counter
    if (lane_change_lock > 0)
    {
        lane_change_lock--;
    }

    if (is_changing_lane)
    {
        // We are changing the lane right now. Do not consider other candidate lanes till the maneuver is finished.
        possible_lanes.push_back(fix_lane);
    }
    else {
        possible_lanes.push_back(start_lane);    // First try to to stay in the current lane. That's most comfortable.
        if (start_lane > 0) {
            // There is at least one lane to the left of our car.
            // Consider changing lane to the left.
            possible_lanes.push_back(start_lane - 1);
        }
        if (start_lane < 2) {
            // There is at least one lane to the right of our car.
            // Consider changing lane to the right.
            possible_lanes.push_back(start_lane + 1);
        }
    }

    // Show debugging info
    char buf[4];
    for (int i = 0; i < possible_lanes.size(); i++)
    {
        buf[i] = possible_lanes[i] + '0';
    }
    char lock = is_changing_lane ? '*' : ' ';
    buf[possible_lanes.size()] = '\0';
    printf("%4d %d%c %3s %2.2f %4.2f %2.2f", iteration, start_lane, lock, buf, t_start_speed, t_start_s, t_start_d);


    double start_time = previous_path_x.size() * 0.02;  // we already have path points for start_time seconds

    double time_frame = kPlanningTime - start_time;     // That's time span that we are going to plan


    double t_target_speed = kMaxSpeed;  // Our initial target speed. We will reduce it later if no feasible trajectory will be found.


    // Instantiate helper classes
    TrajectoryHelper trajectoryHelper(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx,
                                      map_waypoints_dy);
    CollisionChecker collisionChecker(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx,
                                      map_waypoints_dy);
    Trajectory best_trajectory;

    int lane_index = 0; // index in possible_lanes array

    while (t_target_speed > kMinSpeed) {

        // First we calculate acceleration profile ti reach target_speed in time_frame seconds given initial conditions.
        AccelerationProfile profile = SpeedHelper::calculateAccelerationProfile(
                t_start_speed, t_start_acceleration,
                t_target_speed, time_frame,
                kMaxTanAcceleration, kMaxJerk);

        int target_lane = possible_lanes[lane_index];

        // Next we generate a trajectory from start_lane to target_lane (can be the same)
        Trajectory trajectory = trajectoryHelper.buildTrajectory(
                t_start_x, t_start_y, t_start_yaw, t_start_s, t_start_d,
                start_lane, t_start_speed,
                profile, target_lane, time_frame, is_changing_lane);

        // Debug info
        printf("\t\tl:%d v:%2.2f", target_lane, t_target_speed);

        // Do we hit speed or acceleration limit?
        bool is_within_limits = collisionChecker.check_limits(trajectory, kMaxSpeed, kMaxTotalAcceleration);

        bool has_collisions = false;
        if (is_within_limits) {
            // Do we collide with other cars?
            has_collisions = collisionChecker.check_collision(sensor_fusion, t_start_yaw, trajectory, start_time);
        }

        if (has_collisions || !is_within_limits) {

            // current trajectory does is not feasible

            // Debug info
            if (has_collisions) {
                printf("\t\tHit car %2d after %1.2f seconds at s=%4.2f. My d=%2.2f car's d=%2.2f",
                    trajectory.collision_other_car_id,
                    trajectory.collision_time,
                    trajectory.collision_my_s,
                    trajectory.collision_my_d,
                    trajectory.collision_other_d
                    );
            }
            if (!is_within_limits)
            {
                printf("\t\t%s", trajectory.limit_message.c_str());
            }


            if (lane_index < possible_lanes.size() - 1) {
                lane_index++; // try next lane
            } else {
                // All possible lanes tested and no feasible found. Reduce the speed and try again
                lane_index = 0;
                t_target_speed -= 0.447;    // Try to drive 1 MPH slower

                if (t_target_speed < kMinSpeed) {
                    cout << "" << " Emergency situation!" << endl;

                    if (is_changing_lane) {
                        // Continue lane change
                        best_trajectory = trajectoryHelper.buildTrajectory(
                                t_start_x, t_start_y, t_start_yaw, t_start_s, t_start_d, start_lane, t_start_speed,
                                profile, fix_lane, time_frame, is_changing_lane);
                    } else {
                        // Stay in the current lane
                        best_trajectory = trajectoryHelper.buildTrajectory(
                                t_start_x, t_start_y, t_start_yaw, t_start_s, t_start_d, start_lane, t_start_speed,
                                profile, start_lane, time_frame, is_changing_lane);
                    }
                    break;
                }
            }
            cout << endl << "\t\t\t\t\t\t\t";
        } else {
            // trajectory is valid!

            best_trajectory = trajectory;

            if (start_lane != target_lane && !is_changing_lane)
            {
                // Start lane changing maneuver
                fix_lane = target_lane;
                lane_change_lock = 100;    // Fix that decision for next 100 iterations
                cout << " Changing to lane " << target_lane;


/*
                // DEBUGGING!!! Writes logs to file system

                LogHelper::change_lane(car_s, car_d, car_x, car_y,
                                       t_start_s, t_start_d, t_start_x, t_start_y,
                                       t_start_speed, t_start_acceleration, t_start_yaw,
                                       sensor_fusion, best_trajectory, start_lane, target_lane);
*/


            }
            else {
                printf("\t\tKeep lane %d.  accT=%1.1f  accN=%1.1f accTotal=%1.1f",
                        target_lane, trajectory.max_tan_acceleration, trajectory.max_norm_acceleration, trajectory.max_total_acceleration);
            }
            cout << endl;
            break;
        }
    }

    // Copy trajectory's points to output vectors

    for (int i = 0; i < best_trajectory.path_x.size(); i++) {
        next_x_vals.push_back(best_trajectory.path_x[i]);
        next_y_vals.push_back(best_trajectory.path_y[i]);
    }


    vector<double> dx;
    for (int i = 0; i < next_x_vals.size() - 1; i++) {
        dx.push_back((next_x_vals[i + 1] - next_x_vals[i]) / 0.02);
    }
    iteration++;
}



