#ifndef COST_FUNCTIONS_H
#define COST_FUNCTIONS_H

#include <math.h>
#include <string>
#include <vector>
#include <limits>
#include <iostream>

// for convenience
using std::string;
using std::vector;

double logistic(double x) {
    return 2.0 / (1 + exp(-x)) - 1.0;
}

// rewards higher speeds
double intended_speed_cost(double target_speed, double intended_speed) {
    return logistic(2 * (target_speed - intended_speed) / intended_speed);
}

// penalizes if the car gets too close to other cars
double car_proximity_cost(double s, double car_s, double minimum_proximity) {
    if ((s > car_s) && (s - car_s) < minimum_proximity) {
        return logistic(2 * (minimum_proximity - (s - car_s)) / (s - car_s));
    } else {
        return 0.0;
    }
}

double car_in_lane_cost(int intended_lane, int current_lane) {
    if (intended_lane < 0 || intended_lane > 2) {
        return 1.0;
    } else if (intended_lane != current_lane) {
        return 0.1;
    } else {
        return 0.0;
    }
}

double collision_cost_frenet(vector<FrenetCoords> t1, vector<FrenetCoords> t2, double collision_distance) {
    for (int i=0; i < t1.size(); i++) {
        double dist = distanceBetween({ t1[i].s(), t1[i].d() }, { t2[i].s(), t2[i].d() });
        if (dist < collision_distance) {
          //  std::cout << "Colliosion detected because our: " << t1[i].s() << ", " << t1[i].d() << " and other " << t2[i].s() << ", " << t2[i].d() << " and dist = " << dist << "\n";
            return 1.0;
        }
    }

    return 0.0;
}

double driving_outside_lane_center_cost(vector<FrenetCoords> trajectory) {
    int stepCnt = 0;
    
    for (int i=0; i < trajectory.size(); i++) {
        double pos = fmod(trajectory[i].d(), 4);
        if (pos < 1 || pos > 3) {
            stepCnt++;
        }
    }

    return stepCnt / trajectory.size();
}

// rewards bigger distance between cars
double cars_in_front_cost(double s, double car_s) {
    return logistic(2 * (s - car_s) / (s - car_s));
}
#endif