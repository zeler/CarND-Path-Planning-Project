#include "BehavioralModule.hpp"
#include "helpers.h"

State BehavioralModule::getNextState() {
    
    int previous_path_size = previous_path_x.size();

    // if there is any past data, use the last point as the last reference position
    if (previous_path_size > 0) {
        car_s = end_path_s;
    }

    deccelerate = false;

    // iterate through all the cars reported by the sensor fusion
    for (int i = 0; i < sensor_fusion.size(); i++) {
        // [ id, x, y, vx, vy, s, d]
        // find cars in our lane. d is the 7th parameter of the array
        float d = sensor_fusion[i][6];
        // the car is somewhere in our current lane
        if (getCenterOfLaneFrenet(lane) - 2 < d && d < getCenterOfLaneFrenet(lane) + 2) {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double s = sensor_fusion[i][5];
            double speed = sqrt(pow(vx, 2) + pow(vy, 2));

            // project the position of the car in the next frame 
            // we have previous_path_points, so s is behind the reality
            s += (double) previous_path_size * 0.02 * speed; 

            if ((s > car_s) && (s - car_s) < 30) {
                if (lane > 0) {
                    lane = 0;
                } else {
                    deccelerate = true;
                }
            }
        }
    }

    return State { lane, 0, deccelerate };
}

void BehavioralModule::update(double car_s, 
                    double car_d, 
                    double car_speed, 
                    double end_path_s, 
                    double end_path_d, 
                    vector<double> previous_path_x, 
                    vector<double> previous_path_y, 
                    vector<vector<double>> sensor_fusion) {

    this->car_s = car_s;
    this->car_d = car_d;
    this->car_speed = car_speed;
    this->end_path_s = end_path_s;
    this->end_path_d = end_path_d;
    this->previous_path_x = previous_path_x;
    this->previous_path_y = previous_path_y;
    this->sensor_fusion = sensor_fusion;
}