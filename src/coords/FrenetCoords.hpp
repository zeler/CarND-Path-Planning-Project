#ifndef FRENET_COORDS_H
#define FRENET_COORDS_H

#include <vector>
using std::vector;

class FrenetCoords { 
    public:
        FrenetCoords(double car_s, double car_d) : car_s(car_s), car_d(car_d) {};

        double s() { return car_s; }
        double d() { return car_d; }

    private:
        double car_s, car_d;
};

inline vector<vector<double>> split(vector<FrenetCoords> fcs) {
    vector<vector<double>> result(2);

    for (FrenetCoords fc : fcs) {
        result[0].push_back(fc.s());
        result[1].push_back(fc.d());
    }

    return result;
}

#endif