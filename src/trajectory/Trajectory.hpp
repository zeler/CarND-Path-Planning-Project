#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include "../coords/XYCoords.hpp"

using std::vector;

class Trajectory { 
    public:
        Trajectory(vector<XYCoords> coords, double final_velocity) : coords(coords), final_velocity(final_velocity) {};

        vector<XYCoords> xyCoords() { return coords; }
        double finalVelocity() { return final_velocity; }

    private:
        vector<XYCoords> coords;
        double final_velocity;
};

#endif