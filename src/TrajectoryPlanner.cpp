#include "TrajectoryPlanner.hpp"
#include "spline.h"
#include "debug.h"
#include "helpers.h"

//#define ENABLE_COORD_LOG    

#ifdef ENABLE_COORD_LOG
  #define LOG_COORDS(d, x, y)  {  print_coords(d, x, y); }
#else
  #define LOG_COORDS(d, x, y) // do nothing
#endif

vector<vector<double>> TrajectoryPlanner::planTrajectory(TransitionState plan) {
    
    int previous_path_size = previous_path_x.size();
    
    double ref_x_previous;
    double ref_y_previous;
    double ref_x;
    double ref_y;
    double ref_yaw;

    if (previous_path_size > 0) {
        car_s = end_path_s;
    }

    if (previous_path_size < 2) {
        ref_x = car_x;
        ref_y = car_y;
        
        ref_x_previous = car_x - cos(car_yaw);
        ref_y_previous = car_y - sin(car_yaw);

        ref_yaw = deg2rad(car_yaw);
    } else {
        // first set our reference to the last point of previous path
        ref_x = previous_path_x[previous_path_size - 1];
        ref_y = previous_path_y[previous_path_size - 1];

        // calculate ref yaw from last 2 path waypoints
        ref_x_previous = previous_path_x[previous_path_size - 2];
        ref_y_previous = previous_path_y[previous_path_size - 2];
        
        ref_yaw = atan2(ref_y - ref_y_previous, ref_x - ref_x_previous);
    }

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // if we have any points in the previous path, add them now to the path planner
    // this will smooth out the transition
    for (int i = 0; i < previous_path_size; i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    CarState currentState { ref_x_previous, ref_y_previous, ref_x, ref_y, car_s, car_d, ref_yaw, ref_velocity };
    int steps = step_count - previous_path_size;

    vector<vector<double>> new_trajectory = tg.generateTrajectory(currentState, plan, steps);
    
    // add all points from the new trajectory
    for(int i = 0; i < steps; i++) {
        next_x_vals.push_back(new_trajectory[0][i]);
        next_y_vals.push_back(new_trajectory[1][i]);
    }

    this->ref_velocity = new_trajectory[2][0];
    
    return {next_x_vals, next_y_vals};
}

void TrajectoryPlanner::update(double car_x, 
                    double car_y, 
                    double car_s, 
                    double car_d, 
                    double car_yaw, 
                    double car_speed, 
                    double end_path_s, 
                    double end_path_d, 
                    vector<double> previous_path_x, 
                    vector<double> previous_path_y, 
                    vector<vector<double>> sensor_fusion) {

    this->car_x = car_x;
    this->car_y = car_y;
    this->car_s = car_s;
    this->car_d = car_d;
    this->car_yaw = car_yaw;
    this->car_speed = car_speed;
    this->end_path_s = end_path_s;
    this->end_path_d = end_path_d;
    this->previous_path_x = previous_path_x;
    this->previous_path_y = previous_path_y;
    this->sensor_fusion = sensor_fusion;

    LOG_COORDS("PREV_COORDS: ", previous_path_x, previous_path_y);
}