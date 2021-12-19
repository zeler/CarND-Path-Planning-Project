#ifndef BEHAVIORAL_MODULE_H
#define BEHAVIORAL_MODULE_H

#include <vector>
#include <string>

#include "State.hpp"

using std::string;
using std::vector;

class BehavioralModule { 
    public:
        BehavioralModule(double max_speed, int lane) : max_speed(max_speed), lane(lane), target_speed(max_speed) {}
        State getNextState();

        void update(double car_s, 
                    double car_d, 
                    double car_speed, 
                    double end_path_s, 
                    double end_path_d, 
                    vector<double> previous_path_x, 
                    vector<double> previous_path_y, 
                    vector<vector<double>> sensor_fusion);

    private:
        int lane;
        double max_speed;
        double target_speed;

        // updated from simulator
        double car_s, end_path_s, car_d, end_path_d, car_speed;
        vector<double> previous_path_x, previous_path_y;
        vector<vector<double>> sensor_fusion;
};

#endif