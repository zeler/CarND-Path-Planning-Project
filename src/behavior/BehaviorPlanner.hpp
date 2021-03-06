#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <vector>
#include <string>

#include "behavior_consts.hpp"
#include "../states/TransitionState.hpp"
#include "../trajectory/TrajectoryGenerator.hpp"
#include "../coords/CoordsUtils.hpp"
#include "../coords/RefCoords.hpp"
#include "../states/SensedCarState.hpp"
#include "Prediction.hpp"

using std::string;
using std::vector;

class BehaviorPlanner { 
    public:
        BehaviorPlanner(double max_speed, TrajectoryGenerator &tg, CoordsUtils &coordsUtils) 
            : max_speed(max_speed), tg(tg), coordsUtils(coordsUtils) {}
            
        TransitionState getNextState();

        void update(double car_x, 
                    double car_y,
                    double car_s, 
                    double car_d, 
                    double car_yaw,
                    double car_speed, 
                    double end_path_s, 
                    double end_path_d, 
                    vector<double> previous_path_x, 
                    vector<double> previous_path_y, 
                    vector<vector<double>> sensor_fusion);
        
        void reset(double max_speed);

        double maxSpeed() const { return max_speed; }

    private:
        double max_speed;

        // updated from simulator
        double car_x, car_y, car_s, end_path_s, car_d, end_path_d, car_speed, car_yaw;
        vector<double> previous_path_x, previous_path_y;
        vector<vector<double>> sensor_fusion;
        TrajectoryGenerator tg;
        CoordsUtils coordsUtils;

        RefCoords rc;
        
        vector<TransitionState> generatePossibleSuccessorStates(vector<vector<Prediction>> predictions, CarState cs);
        vector<TransitionState> getStatesForLane(int lane, double car_s, vector<vector<Prediction>> predictions);
        vector<vector<Prediction>> predict(vector<vector<SensedCarState>> relevant_cars);
        TransitionState chooseNextState(vector<vector<Prediction>> predictions);
};

#endif