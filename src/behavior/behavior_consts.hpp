#ifndef BEHAVIOR_CONSTS_H
#define BEHAVIOR_CONSTS_H

const double maximum_relevant_distance = 100;
const double maximum_relevant_distance_behind = 20;
const double min_proximity = 6.5;
const double collision_distance = 3.8;
const int prediction_step_count = 100;

const double collision_cost_weight = 5.0;
const double car_proximity_cost_weight = 0.9;

const double car_in_lane_cost_weight = 0.01;
const double driving_outside_lane_center_cost_weight = 0.0;

const double intended_speed_cost_weight = 0.65; 
const double cars_in_front_cost_weight = 0.01f;

#endif