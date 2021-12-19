#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <vector>
#include <string>

#include "State.hpp"

using std::string;
using std::vector;

class TrajectoryPlanner { 

    public:
        TrajectoryPlanner(
            vector<double> &map_waypoints_s, 
            vector<double> &map_waypoints_x, 
            vector<double> &map_waypoints_y)
            : 
            map_waypoints_s(map_waypoints_s),
            map_waypoints_x(map_waypoints_x),
            map_waypoints_y(map_waypoints_y),
            ref_velocity(0) {}

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

        vector<vector<double>> planTrajectory(State nextState);
    
    private:
        double car_x, car_y, car_s, car_d, car_yaw, car_speed, end_path_s, end_path_d, ref_velocity;
        vector<double> map_waypoints_s, map_waypoints_x, map_waypoints_y, previous_path_x, previous_path_y;
        vector<vector<double>> sensor_fusion;

        const double speed_delta = 0.224;
};

#endif