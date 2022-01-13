#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <vector>
#include <string>

#include "TrajectoryGenerator.hpp"
#include "../states/TransitionState.hpp"
#include "../behavior/BehaviorPlanner.hpp"
#include "../coords/CoordsUtils.hpp"

using std::string;
using std::vector;

class TrajectoryPlanner { 

    public:
        TrajectoryPlanner(TrajectoryGenerator &tg, BehaviorPlanner &bp, CoordsUtils &coordsUtils) : tg(tg), bp(bp), coordsUtils(coordsUtils), ref_velocity(0), plan({1, bp.maxSpeed()}) {}

        void update(double car_x, 
                    double car_y, 
                    double car_s, 
                    double car_d, 
                    double car_yaw, 
                    double car_speed, 
                    double end_path_s, 
                    double end_path_d, 
                    vector<double> previous_path_x, 
                    vector<double> previous_path_y, 
                    vector<vector<double>> sensor_fusion);

        vector<XYCoords> planTrajectory();

        void setRefVelocity(double refVelocity) { ref_velocity = refVelocity; }
    
    private:
        double car_x, car_y, car_s, car_d, car_yaw, car_speed, end_path_s, end_path_d, ref_velocity;
        vector<double> previous_path_x, previous_path_y;
        vector<vector<double>> sensor_fusion;
        TrajectoryGenerator tg;
        BehaviorPlanner bp;
        CoordsUtils coordsUtils;

        TransitionState plan;

        //const double speed_delta = 0.224;
        const double speed_delta = 0.45;
        const int step_count = 25;
};

#endif