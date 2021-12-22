#include "TrajectoryGenerator.hpp"
#include "spline.h"
#include "debug.h"
#include "helpers.h"

//#define ENABLE_COORD_LOG    
//#define ENABLE_SPLINE_LOG

#ifdef ENABLE_COORD_LOG
  #define LOG_COORDS(d, x, y)  {  print_coords(d, x, y); }
#else
  #define LOG_COORDS(d, x, y) // do nothing
#endif

#ifdef ENABLE_SPLINE_LOG
  #define LOG_SPLINE_COORDS(d, x, y)  {  print_coords(d, x, y); }
#else
  #define LOG_SPLINE_COORDS(d, x, y) // do nothing
#endif

vector<vector<double>> TrajectoryGenerator::generateTrajectory(CarState from, TransitionState plan, int steps) {
    
    vector<double> sparse_x;
    vector<double> sparse_y;
    double ref_x = from.carX();
    double ref_y = from.carY();
    double ref_yaw = from.carYaw();
    double car_s = from.carS();
    double ref_velocity = from.carSpeed();
    
    sparse_x.push_back(from.prevCarX());
    sparse_x.push_back(ref_x);
    sparse_y.push_back(from.prevCarY());
    sparse_y.push_back(ref_y);

    // add extra 3 points, each 30 metres apart (30, 60. 90 m from actual car_s). see sparse_point_distance
    for (int x = 1; x < 4; x++) {
        vector<double> xy = getXY(car_s + x * sparse_point_distance, getCenterOfLaneFrenet(plan.getLane()), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        sparse_x.push_back(xy[0]);
        sparse_y.push_back(xy[1]);
    }

    LOG_SPLINE_COORDS("WORLD_COORDS: ", sparse_x, sparse_y);

    for (int i = 0; i < sparse_x.size(); i++) {
        // set car reference angle to 0 deg for an easier math later
        // euclidean shift
        double shift_x = sparse_x[i] - ref_x;
        double shift_y = sparse_y[i] - ref_y;

        // euclidean rotation
        sparse_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        sparse_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    LOG_SPLINE_COORDS("LOCAL_COORDS: ", sparse_x, sparse_y);

    // build spline
    tk::spline s;
    s.set_points(sparse_x, sparse_y);

    // speed calculation based on FAQ video
    double target_x = sparse_point_distance;
    double target_y = s(target_x);
    // pythagoras theorem
    double target_d = sqrt(pow(target_x, 2) + pow(target_y, 2));
    double x_add_on = 0;
    
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // add new points to the path
    for (int i = 0; i < steps; i++) {
        if (ref_velocity >= plan.targetSpeed()) {
            ref_velocity -= speed_delta;
        } else if (ref_velocity < plan.targetSpeed()) {
            ref_velocity += speed_delta;
        }
       
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

    // along with new coordinates, we need to report the velocity we achieved. car_speed reported by the sim is unreliable.
    return {next_x_vals, next_y_vals, {ref_velocity}};
}