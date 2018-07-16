//
// Created by victor on 16.07.18.
//

#ifndef PATH_PLANNING_MAPTRANSFORMER_H
#define PATH_PLANNING_MAPTRANSFORMER_H

#include <vector>
using namespace std;


class MapTransformer {
public:
    static vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
    static vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
    static double deg2rad(double x);
    static double rad2deg(double x);
};

#endif //PATH_PLANNING_MAPTRANSFORMER_H
