#include "BehaviorPlanner.hpp"
#include "helpers.h"
#include "debug.h"

#define ENABLE_STATE_LOG    

#ifdef ENABLE_STATE_LOG
  #define LOG_STATE(d, s)  {  print_state(d, s); }
#else
  #define LOG_STATE(d, s) // do nothing
#endif

TransitionState BehaviorPlanner::getNextState() {
    
    int previous_path_size = previous_path_x.size();

    // if there is any past data, use the last point as the last reference position
    if (previous_path_size > 0) {
        car_s = end_path_s;
    }

    // used to store indexes of relevant cards per lane
    // lane[relevant_car[s, d, speed, vx, vy]]
    vector<vector<vector<double>>> relevant_cars(3);

    // iterate through all the cars reported by the sensor fusion and filter relevant cars for each lane
    for (int i = 0; i < sensor_fusion.size(); i++) {
        // [ id, x, y, vx, vy, s, d]
        // find cars in our lane. d is the 7th parameter of the array
        float d = sensor_fusion[i][6];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double s = sensor_fusion[i][5];
        double speed = sqrt(pow(vx, 2) + pow(vy, 2));

        // project the position of the car in the next frame 
        // we have previous_path_points, so s is behind the reality
        s += (double) previous_path_size * 0.02 * speed; 
        
        // if the car is in relevant distance to our car
        // TODO: is this the same also for faster cars in other lanes?
        if ((s > car_s) && (s - car_s) < maximum_relevant_distance) {
            // find the lane of the car
            for (int x = 0; x < 3; x++) {
                if (getCenterOfLaneFrenet(x) - 2 < d && d < getCenterOfLaneFrenet(x) + 2) {
                    vector<double> car {s, d, speed, vx, vy};
                    relevant_cars[x].push_back(car);
                    break;
                }
            }
        }
    }

    if(!relevant_cars[lane].empty()) {
        
        if (lane > 0) {
            lane--;
        } 
        // else if (lane < 3) {
        //     lane++;
        // } 
        else {
            // get the speed of the first car in front of us
            target_speed = relevant_cars[lane][0][2]  * 2.24;
        }
    }
     
    TransitionState next_state { lane, target_speed};
    LOG_STATE("Next state", next_state);

    return next_state;
}

void BehaviorPlanner::update(double car_s, 
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

void BehaviorPlanner::reset(double max_speed, int lane) {
    this->max_speed = max_speed;
    this->lane = lane;
    this->target_speed = target_speed;
}