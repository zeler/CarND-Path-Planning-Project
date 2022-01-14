#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include <string>

#include "../states/CarState.hpp"
#include "../states/SensedCarState.hpp"
#include "../states/TransitionState.hpp"
#include "../coords/RefCoords.hpp"
#include "../coords/XYCoords.hpp"
#include "../coords/FrenetCoords.hpp"
#include "Trajectory.hpp"
#include "../coords/CoordsUtils.hpp"

using std::string;
using std::vector;

class TrajectoryGenerator { 

    public:
        TrajectoryGenerator(vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, CoordsUtils cu)
                    : map_waypoints_s(map_waypoints_s), map_waypoints_x(map_waypoints_x), map_waypoints_y(map_waypoints_y), cu(cu) {}
        
        Trajectory generateSplineTrajectory(CarState from, TransitionState to, int steps);

        vector<FrenetCoords> generateLineTrajectory(SensedCarState from, int steps);
    
    private:
        vector<double> map_waypoints_s, map_waypoints_x, map_waypoints_y;
        CoordsUtils cu;

        //const double speed_delta = 0.224;
        const double speed_delta = 0.3;
        const double sparse_point_distance = 45;
};

#endif