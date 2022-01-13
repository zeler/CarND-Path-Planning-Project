#include "TrajectoryPlanner.hpp"
#include "../utils/debug.h"
#include "../utils/helpers.h"
#include "../coords/RefCoords.hpp"

//#define ENABLE_COORD_LOG    

#ifdef ENABLE_COORD_LOG
  #define LOG_COORDS(d, x, y)  {  print_coords(d, x, y); }
#else
  #define LOG_COORDS(d, x, y) // do nothing
#endif

vector<XYCoords> TrajectoryPlanner::planTrajectory() {

    plan = bp.getNextState();
    
    int previous_path_size = previous_path_x.size();
    RefCoords rc = coordsUtils.toRefCoords(car_x, car_y, car_yaw, previous_path_x, previous_path_y, car_s, car_d, end_path_s, end_path_d);

    vector<XYCoords> next_vals;

    // if we have any points in the previous path, add them now to the path planner
    // this will smooth out the transition
    for (int i = 0; i < previous_path_size; i++) {
        next_vals.push_back({previous_path_x[i], previous_path_y[i]});
    }

    CarState currentState { rc.refXPrevious(), rc.refYPrevious(), rc.refX(), rc.refY(), rc.refS(), rc.refD(), rc.refYaw(), ref_velocity };
    int steps = step_count - previous_path_size;

    Trajectory new_trajectory = tg.generateSplineTrajectory(currentState, plan, steps);
    
    // add all points from the new trajectory
    for(int i = 0; i < steps; i++) {
        next_vals.push_back({new_trajectory.xyCoords()[i].x(), new_trajectory.xyCoords()[i].y()});
    }

    this->ref_velocity = new_trajectory.finalVelocity();
    
    return next_vals;
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

    double planned_center_of_lane = getCenterOfLaneFrenet(plan.getLane());

    // we need to do this update here, as car speed information from the sim is unreliable
    bp.update(car_x, car_y, car_s, car_d, car_yaw, ref_velocity, end_path_s, end_path_d, previous_path_x, previous_path_y, sensor_fusion);

    LOG_COORDS("PREV_COORDS: ", previous_path_x, previous_path_y);
}