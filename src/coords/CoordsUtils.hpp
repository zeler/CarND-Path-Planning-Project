#ifndef COORDS_UTILS_H
#define COORDS_UTILS_H

#include <vector>
#include <string>

#include "../states/CarState.hpp"
#include "../states/TransitionState.hpp"
#include "RefCoords.hpp"
#include "XYCoords.hpp"
#include "FrenetCoords.hpp"
#include "../trajectory/Trajectory.hpp"

using std::string;
using std::vector;

class CoordsUtils { 

    public:
        CoordsUtils(vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y)
                    : map_waypoints_s(map_waypoints_s), map_waypoints_x(map_waypoints_x), map_waypoints_y(map_waypoints_y) {}
        
        XYCoords convertToXY(FrenetCoords fc);

        vector<FrenetCoords> convertToFrenet(vector<XYCoords> xyCoords, double theta);

        RefCoords toRefCoords(double car_x, double car_y, double car_yaw, vector<double> previous_path_x, vector<double> previous_path_y, 
                                double car_s, double car_d, double end_path_s, double end_path_d);

        RefCoords toRefCoordsFromCurrent(double car_x, double car_y, double car_yaw, double car_s, double car_d);
    
    private:
        vector<double> map_waypoints_s, map_waypoints_x, map_waypoints_y;
};
#endif