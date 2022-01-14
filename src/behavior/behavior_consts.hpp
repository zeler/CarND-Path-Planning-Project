#ifndef BEHAVIOR_CONSTS_H
#define BEHAVIOR_CONSTS_H

const double maximum_relevant_distance = 100;
const double maximum_relevant_distance_behind = 25;
const double min_proximity = 4;
const double collision_distance = 3.4;
const double collision_in_s_distance_min_d = 2.8;
const double collision_in_s_distance = 4;
const int prediction_step_count = 100;

const double collision_cost_weight = 5.0; 
const double collision_cost_in_s_weight = 5.0;
const double car_proximity_cost_weight = 0.1;

const double car_in_lane_cost_weight = 0.1;
const double driving_outside_lane_center_cost_weight = 5.0;

const double intended_speed_cost_weight = 0.002; 
const double cars_in_front_cost_weight = 0.04f;

#endif