#ifndef PREDICTION_H
#define PREDICTION_H

#include <vector>
#include "../coords/FrenetCoords.hpp"
#include "../states/SensedCarState.hpp"

using std::vector;

class Prediction { 
    public:
        Prediction(SensedCarState carState, vector<FrenetCoords> prediction) : carState(carState), prediction(prediction) {};

        SensedCarState sensedCarState() { return carState; }
        vector<FrenetCoords> predictedTrajectory() { return prediction; }

    private:
        SensedCarState carState;
        vector<FrenetCoords> prediction;
};

#endif