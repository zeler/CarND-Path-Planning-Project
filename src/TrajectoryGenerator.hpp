#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include <string>

#include "CarState.hpp"
#include "TransitionState.hpp"

using std::string;
using std::vector;

class TrajectoryGenerator { 

    public:
        TrajectoryGenerator(vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y)
                    : map_waypoints_s(map_waypoints_s), map_waypoints_x(map_waypoints_x), map_waypoints_y(map_waypoints_y) {}
        
        vector<vector<double>> generateTrajectory(CarState from, TransitionState to, int steps);
    
    private:
        vector<double> map_waypoints_s, map_waypoints_x, map_waypoints_y;

        const double speed_delta = 0.224;
        const double sparse_point_distance = 30;
};

#endif