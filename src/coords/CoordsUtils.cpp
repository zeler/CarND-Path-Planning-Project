#include "CoordsUtils.hpp"
#include "../utils/spline.h"
#include "../utils/debug.h"
#include "../utils/helpers.h"

XYCoords CoordsUtils::convertToXY(FrenetCoords fc) {
    vector<double> xy = getXY(fc.s(), fc.d(), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    return { xy[0], xy[1] };
}

vector<FrenetCoords> CoordsUtils::convertToFrenet(vector<XYCoords> xyCoords, double theta) {
    vector<FrenetCoords> frenet;

    for (int i = 0; i < xyCoords.size(); i++) {
        vector<double> coords = getFrenet(xyCoords[i].x(), xyCoords[i].y(), theta, map_waypoints_x, map_waypoints_y);
        frenet.push_back({coords[0], coords[1]});
    }

    return frenet;
}

// extracts correct ref vals for later calculations
RefCoords CoordsUtils::toRefCoords(double car_x, double car_y, double car_yaw, vector<double> previous_path_x, vector<double> previous_path_y, 
                                double car_s, double car_d, double end_path_s, double end_path_d) {
    
    double ref_x_previous;
    double ref_y_previous;
    double ref_x;
    double ref_y;
    double ref_yaw;
    double ref_s;
    double ref_d;

    int previous_path_size = previous_path_x.size();

    if (previous_path_size < 2) {
        ref_x = car_x;
        ref_y = car_y;
        
        ref_x_previous = car_x - cos(car_yaw);
        ref_y_previous = car_y - sin(car_yaw);

        ref_yaw = deg2rad(car_yaw);
        
        ref_s = car_s;
        ref_d = car_d;
    } else {
        // first set our reference to the last point of previous path
        ref_x = previous_path_x[previous_path_size - 1];
        ref_y = previous_path_y[previous_path_size - 1];

        // calculate ref yaw from last 2 path waypoints
        ref_x_previous = previous_path_x[previous_path_size - 2];
        ref_y_previous = previous_path_y[previous_path_size - 2];
        
        ref_yaw = getCarYaw(ref_x_previous, ref_x, ref_y_previous, ref_y);

        ref_s = end_path_s;
        ref_d = end_path_d;
    }

    return {ref_x_previous, ref_y_previous, ref_x, ref_y, ref_yaw, ref_s, ref_d};
}

// extracts correct ref vals for later calculations
RefCoords CoordsUtils::toRefCoordsFromCurrent(double car_x, double car_y, double car_yaw, double car_s, double car_d) {
    
    XYCoords previous = convertToXY({car_s - 30, car_d - 30});

    double ref_x_previous = previous.x();
    double ref_y_previous = previous.y();
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);
    double ref_s = car_s;
    double ref_d = car_d;

    return {ref_x_previous, ref_y_previous, ref_x, ref_y, ref_yaw, ref_s, ref_d};
}