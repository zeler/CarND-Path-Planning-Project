#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <vector>
#include <string>

#include "TransitionState.hpp"

using std::string;
using std::vector;

class BehaviorPlanner { 
    public:
        BehaviorPlanner(double max_speed, int lane) : max_speed(max_speed), lane(lane), target_speed(max_speed) {}
        TransitionState getNextState();

        void update(double car_s, 
                    double car_d, 
                    double car_speed, 
                    double end_path_s, 
                    double end_path_d, 
                    vector<double> previous_path_x, 
                    vector<double> previous_path_y, 
                    vector<vector<double>> sensor_fusion);
        
        void reset(double max_speed, int lane);
    private:
        int lane;
        double max_speed;
        double target_speed;

        // updated from simulator
        double car_s, end_path_s, car_d, end_path_d, car_speed;
        vector<double> previous_path_x, previous_path_y;
        vector<vector<double>> sensor_fusion;

        const double maximum_relevant_distance = 30;
};

#endif