#include "BehaviorPlanner.hpp"
#include "../utils/helpers.h"
#include "../utils/debug.h"
#include "cost_functions.hpp"
#include "../coords/RefCoords.hpp"

using std::begin;
using std::end;

//#define ENABLE_STATE_LOG    

#ifdef ENABLE_STATE_LOG
  #define LOG_STATE(d, s)  {  print_state(d, s); }
  #define LOG_SUCCESSOR_STATES(d, s) { print_state_vector(d, s); }
  #define LOG_COSTS(d, v) { print_vector(d, v); }
#else
  #define LOG_STATE(d, s) // do nothing
  #define LOG_SUCCESSOR_STATES(d, s) 
  #define LOG_COSTS(d, v)
#endif

TransitionState BehaviorPlanner::getNextState() {
    
    rc = coordsUtils.toRefCoords(car_x, car_y, car_yaw, previous_path_x, previous_path_y, car_s, car_d, end_path_s, end_path_d);
    //rc = coordsUtils.toRefCoordsFromCurrent(car_x, car_y, car_yaw, car_s, car_d);
    int previous_path_size = previous_path_x.size();

    // used to store indexes of relevant cards per lane
    // lane[relevant_car[s, d, speed, vx, vy]]
    vector<vector<SensedCarState>> relevant_cars_by_lane(3);

    // iterate through all the cars reported by the sensor fusion and filter relevant cars for each lane
    for (int i = 0; i < sensor_fusion.size(); i++) {
        // [ id, x, y, vx, vy, s, d]
        // find cars in our lane. d is the 7th parameter of the array
        double x = sensor_fusion[i][1];
        double y = sensor_fusion[i][2];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double s = sensor_fusion[i][5];
        double d = sensor_fusion[i][6];
        double speed = sqrt(pow(vx, 2) + pow(vy, 2));

        // project the position of the car after the planned trajectory
        s += (double) previous_path_size * 0.02 * speed; 
     
        // find the lane of the car
        if (d > 0  
            && ((s > rc.refS()) && (s - rc.refS()) <= maximum_relevant_distance) 
                || ((s < rc.refS()) && (rc.refS() - s) <= maximum_relevant_distance_behind)) {
            relevant_cars_by_lane[deriveLane(d)].push_back({s, d, speed});
        }
    }

//    std::cout << "Cars in lane 0: " << relevant_cars_by_lane[0].size() << "\n";
//    std::cout << "Cars in lane 1: " << relevant_cars_by_lane[1].size() << "\n";
//    std::cout << "Cars in lane 2: " << relevant_cars_by_lane[2].size() << "\n";
    
    vector<vector<Prediction>> predictions = predict(relevant_cars_by_lane);
    TransitionState chosen_state = chooseNextState(predictions);
    LOG_STATE("Chosen state", chosen_state);

    return chosen_state;
}

vector<vector<Prediction>> BehaviorPlanner::predict(vector<vector<SensedCarState>> relevant_cars) {
     
    vector<vector<Prediction>> predictions(relevant_cars.size());
     
    for (int i = 0; i < relevant_cars.size(); i++) {
        for (int k = 0; k < relevant_cars[i].size(); k++) {
            predictions[i].push_back({relevant_cars[i][k], tg.generateLineTrajectory(relevant_cars[i][k], prediction_step_count)});
        }
    }

    return predictions;
}

TransitionState BehaviorPlanner::chooseNextState(vector<vector<Prediction>> predictions) {

    CarState state { rc.refXPrevious(), rc.refYPrevious(), rc.refX(), rc.refY(), rc.refS(), rc.refD(), rc.refYaw(), car_speed};

    vector<TransitionState> successor_states = generatePossibleSuccessorStates(predictions, state);
    //std::cout << "Our S: " << rc.refS() << " Our D: " << rc.refD() << "\n";

    LOG_SUCCESSOR_STATES("Successor states: ", successor_states);

    vector<double> costs;

    for (TransitionState ss : successor_states) {

        vector<TransitionState> cts = { ss };
        //LOG_SUCCESSOR_STATES("State vector: ", cts);
       
        vector<FrenetCoords> state_trajectory = coordsUtils.convertToFrenet(tg.generateSplineTrajectory(state, ss, prediction_step_count).xyCoords(), rc.refYaw());
 
        double cost = intended_speed_cost_weight * intended_speed_cost(max_speed, ss.targetSpeed())
                         + car_in_lane_cost_weight * car_in_lane_cost(ss.getLane(), state.carLane())
                         + driving_outside_lane_center_cost_weight * driving_outside_lane_center_cost(state_trajectory);

        for (int i = 0; i < predictions.size(); i++) {
            for (int k = 0; k < predictions[i].size(); k++) {          
                vector<FrenetCoords> predictedTraj = predictions[i][k].predictedTrajectory();
                double collision_cost = collision_cost_frenet(state_trajectory, predictedTraj, collision_distance);
                cost += collision_cost_weight * collision_cost;

                double collision_cost_in_s = collision_cost_in_s_frenet(state_trajectory, predictedTraj, collision_in_s_distance, collision_in_s_distance_min_d);
                cost += collision_cost_in_s_weight * collision_cost_in_s; 
            }
        }
        
        if (predictions[ss.getLane()].size() > 0) {
            double stateFinalS = state_trajectory[state_trajectory.size() - 1].s();
            bool closestCarAccounted = false;

            for (int i = 0; i < predictions[ss.getLane()].size(); i++) {
                vector<FrenetCoords> predictedFinalTraj = predictions[ss.getLane()][i].predictedTrajectory();
                
                double proximity_cost = car_proximity_cost(predictedFinalTraj, state_trajectory, min_proximity);
                cost +=  car_proximity_cost_weight * proximity_cost;

                double predictedFinalS = predictedFinalTraj[predictedFinalTraj.size() - 1].s();
                if (predictedFinalS > stateFinalS && !closestCarAccounted) {
                 //   std::cout << "in " << predictions[ss.getLane()].size() << "\n";
                    cost += cars_in_front_cost_weight * cars_in_front_cost(predictedFinalS, stateFinalS);
                    closestCarAccounted = true;
                }
            }

            
        }

        costs.push_back(cost);
    }

    LOG_COSTS("Costs: ", costs);

    int lowest_index = 0;
    double lowest_cost = costs[0];

    for (int i = 1; i < costs.size(); i++) {
        if (costs[i] < lowest_cost) {
            lowest_index = i;
            lowest_cost = costs[i];
        }
    }

    return successor_states[lowest_index];
}

vector<TransitionState> BehaviorPlanner::generatePossibleSuccessorStates(vector<vector<Prediction>> predictions, CarState cs) {  
    vector<TransitionState> successor_states;
    int lane = cs.carLane();

    for (int i = 0; i < 3; i++) {
        //if (lane + i >= 0 && lane + i <= 2) {
            vector<TransitionState> states = getStatesForLane(/*lane +*/ i, cs.carS(), predictions);
            successor_states.insert(end(successor_states), begin(states), end(states));
        //}
    }

    return successor_states;
}

vector<TransitionState> BehaviorPlanner::getStatesForLane(int lane, double car_s, vector<vector<Prediction>> predictions) {
    vector<Prediction> cars_in_lane = predictions[lane];
    vector<TransitionState> states;

    // nothing in front of us, create state at max speed
    states.push_back(TransitionState {lane, max_speed});

    for (int i = 0; i < cars_in_lane.size(); i++) {
        if (cars_in_lane[i].sensedCarState().s() >= car_s
            && (cars_in_lane[i].sensedCarState().s() - car_s) < maximum_relevant_distance
            && cars_in_lane[i].sensedCarState().speed() <= max_speed) {
            // if there is a car in front of us, an option would be to slow down
            states.push_back(TransitionState {lane, cars_in_lane[i].sensedCarState().speed()});
            states.push_back(TransitionState {lane, cars_in_lane[i].sensedCarState().speed() - 4});

            if (lane > 0) {
                states.push_back(TransitionState {lane - 1, cars_in_lane[i].sensedCarState().speed()});
                states.push_back(TransitionState {lane - 1, cars_in_lane[i].sensedCarState().speed() - 4});
            }

            if (lane < 2) {
                states.push_back(TransitionState {lane + 1, cars_in_lane[i].sensedCarState().speed()});
                states.push_back(TransitionState {lane + 1, cars_in_lane[i].sensedCarState().speed() - 4});
            }

            break;
        }
    }
    
    return states;        
}

void BehaviorPlanner::update(double car_x,
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
}

void BehaviorPlanner::reset(double max_speed) {
    this->max_speed = max_speed;
}