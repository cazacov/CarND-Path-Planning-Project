//
// Created by victor on 29.07.18.
//

#include "trajectory.h"
#include <math.h>

void Trajectory::update_metrics(double time_step)
{
    path_c.clear();
    if (path_x.size() < 3)
    {
        for (int i = 0; i < path_x.size(); i++)
        {
            path_c.push_back(0);
        }
        return;
    }

    path_c.push_back(0);
    for (int i = 0; i < path_x.size() - 2; i++)
    {
        double dx1 = path_x[i + 1] - path_x[i];
        double dx2 = path_x[i + 2] - path_x[i-1];
        double dx3 = path_x[i + 2] - path_x[i];
        double dy1 = path_y[i + 1] - path_y[i];
        double dy2 = path_y[i + 2] - path_y[i-1];
        double dy3 = path_y[i + 2] - path_y[i];

        double mag_1 = sqrt(dx1*dx1 + dy1*dy1);
        double mag_2 = sqrt(dx2*dx2 + dy2*dy2);
        double mag_3 = sqrt(dx3*dx3 + dy3*dy3);

        if (mag_1 < 0.0001 || mag_2 < 0.0001)
        {
            // cannot calculate
            path_c.push_back(1000000);  // some very big number
        }
        else {
            double angle = acos((dx1*dx2 + dy1*dy2) / mag_1 / mag_2);
            double c = 2 * sin(angle) / mag_3;
            path_c.push_back(c);
        }
    }
    path_c.push_back(0);

    // calculate tangential velocity
    max_speed = path_v[0];
    path_v_p.clear();
    path_v_p.push_back(path_v[0]);
    for (int i = 0; i < path_x.size()-1; i++)
    {
        double dx = path_x[i + 1] - path_x[i];
        double dy = path_y[i + 1] - path_y[i];
        double v = sqrt(dx*dx+dy*dy) / time_step;
        path_v_p.push_back(v);
        if (max_speed < v)
        {
            max_speed = v;
        }
    }

    // calculate normal acceleration
    mean_normal_acceleration = 0.0;
    max_normal_acceleration = 0.0;
    for (int i = 0; i < path_v_p.size(); i++)
    {
        double acc_norm = path_v_p[i] * path_v_p[i] * path_k[i];
        if (acc_norm > max_normal_acceleration)
        {
            max_normal_acceleration = acc_norm;
        }
        mean_normal_acceleration += acc_norm;
    }
    mean_normal_acceleration = mean_normal_acceleration /  path_v_p.size();
}
