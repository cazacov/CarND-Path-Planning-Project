#include "log_helper.h"
#include <chrono>
#include <stdio.h>
#include <math.h>
#include "maptransformer.h"

using namespace std;

void
LogHelper::change_lane(double car_s, double car_d, double car_x, double car_y, double start_s, double start_d, double start_x,
                       double start_y, double start_speed, double start_acceleration, double start_yaw,
                       std::vector<std::vector<double>> &sensor_fusion, Trajectory &trajectory, int from_lane, int to_lane) {


    std::time_t t = std::time(0);   // get time now
    std::tm *now = std::localtime(&t);

    char filename[100];

    sprintf(filename, "/home/victor/Udacity/CarND3/CarND-Path-Planning-Project/logs/%02d-%02d-%02d.txt", now->tm_hour, now->tm_min, now->tm_sec);

    FILE *log_file = fopen(filename, "w");
    fprintf(log_file, "car_s: %4.2f   car_d: %2.2f  car_x: %4.2f   car_y: %4.2f\n", car_s, car_d, car_x, car_y);
    fprintf(log_file, "start_s: %4.2f   start_d: %2.2f  start_x: %4.2f   start_y: %4.2f\n", start_s, start_d, start_x, start_y);
    fprintf(log_file, "start_speed: %2.2f   start_acc: %2.2f  start_yaw: %1.3f\n\n", start_speed, start_acceleration, start_yaw);

    fprintf(log_file, "Change lane from %d to %d\n\n", from_lane, to_lane);

    for (int car = 0; car < sensor_fusion.size(); car++) {
        int car_id = sensor_fusion[car][0];
        double car_x = sensor_fusion[car][1];
        double car_y = sensor_fusion[car][2];
        double car_vx = sensor_fusion[car][3];
        double car_vy = sensor_fusion[car][4];
        double car_v = sqrt(car_vx * car_vx + car_vy * car_vy);

        double car_s = sensor_fusion[car][5];
        double car_d = sensor_fusion[car][6];
        int car_lane = MapTransformer::d2lane(car_d);

        fprintf(log_file, "Car %d  x:%4.2f  y:%4.2f  vx: %2.2f vy: %2.2f  v: %2.2f  s0: %4.2f  d0: %4.2f\n",
                car_id, car_x, car_y, car_vx, car_vy, car_v, car_s, car_d);

    }

    for (int i = 0; i < trajectory.my_sd.size(); i++)
    {
        fprintf(log_file, "S: %4.2f D: %2.2f V: %2.2f ", trajectory.my_sd[i][0], trajectory.my_sd[i][1], trajectory.my_sd[i][2]);

        for (int j = 0; j < sensor_fusion.size(); j++)
        {
            fprintf(log_file, "\ts: %4.2f d: %2.2f ", trajectory.cars_sd[i][j*2], trajectory.cars_sd[i][j*2+1]);
        }
        fprintf(log_file,"\n");
    }

    fclose(log_file);
}
