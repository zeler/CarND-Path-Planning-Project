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

vector<vector<double>> TrajectoryPlanner::planTrajectory(State plan) {
    
    // this should happen in path planner and plan this only once for the new points 
    if (plan.isDeccelerate()) {
        // 5 m/s^2
        if (ref_velocity >= 0.448)
            ref_velocity -= 0.224;
    } else if (ref_velocity < max_speed) {
        ref_velocity += 0.224;
    }

    int previous_path_size = previous_path_x.size();
    vector<double> sparse_x;
    vector<double> sparse_y;
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    if (previous_path_size > 0) {
        car_s = end_path_s;
    }

    if (previous_path_size < 2) {
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        sparse_x.push_back(prev_car_x);
        sparse_x.push_back(car_x);
        sparse_y.push_back(prev_car_y);
        sparse_y.push_back(car_y);
    } else {
        // first set our reference to the last point of previous path
        ref_x = previous_path_x[previous_path_size - 1];
        ref_y = previous_path_y[previous_path_size - 1];

        // calculate ref yaw from last 2 path waypoints
        double ref_x_previous = previous_path_x[previous_path_size - 2];
        double ref_y_previous = previous_path_y[previous_path_size - 2];
        ref_yaw = atan2(ref_y - ref_y_previous, ref_x - ref_x_previous);

        sparse_x.push_back(ref_x_previous);
        sparse_x.push_back(ref_x);
        sparse_y.push_back(ref_y_previous);
        sparse_y.push_back(ref_y);            
    }

    // add extra 3 points, each 30 metres apart (30, 60. 90 m from actual car_s). 
    for (int x = 1; x < 4; x++) {
        vector<double> xy = getXY(car_s + x * 30, getCenterOfLaneFrenet(plan.getLane()), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        sparse_x.push_back(xy[0]);
        sparse_y.push_back(xy[1]);
    }

    LOG_COORDS("WORLD_COORDS: ", sparse_x, sparse_y);

    for (int i = 0; i < sparse_x.size(); i++) {
    // set car reference angle to 0 deg for an easier math later
    // euclidean shift
    double shift_x = sparse_x[i] - ref_x;
    double shift_y = sparse_y[i] - ref_y;

    // euclidean rotation
    sparse_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    sparse_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    LOG_COORDS("LOCAL_COORDS: ", sparse_x, sparse_y);

    // build spline
    tk::spline s;
    s.set_points(sparse_x, sparse_y);

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // if we have any points in the previous path, add them now to the path planner
    // this will smooth out the transition
    for (int i = 0; i < previous_path_size; i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    double target_x = 30.0;
    double target_y = s(target_x);
    // pythagoras theorem
    double target_d = sqrt(pow(target_x, 2) + pow(target_y, 2));

    double x_add_on = 0;

    // add new points to the path
    for (int i = 0; i < 50 - previous_path_size; i++) {
       
        // d = N * 0.02 (car advances every 20 ms) * velocity
        double N = (target_d / (0.02 * ref_velocity / 2.24));
        double x_point = x_add_on + target_x / N;
        double y_point = s(x_point);
        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotate back (inverse to previous step)
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        // shift back to origin
        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

     LOG_COORDS("NEW_COORDS: ", next_x_vals, next_y_vals);

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