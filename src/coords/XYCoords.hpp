#ifndef XY_COORDS_H
#define XY_COORDS_H

#include <vector>
using std::vector;

class XYCoords { 
    public:
        XYCoords(double car_x, double car_y) : car_x(car_x), car_y(car_y) {};

        double x() { return car_x; }
        double y() { return car_y; }

    private:
        double car_x,car_y;
};

inline vector<vector<double>> split(vector<XYCoords> xys) {
    vector<vector<double>> result(2);

    for (XYCoords xy : xys) {
        result[0].push_back(xy.x());
        result[1].push_back(xy.y());
    }

    return result;
}


#endif