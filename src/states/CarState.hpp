#ifndef CAR_STATE_H
#define CAR_STATE_H

#include "../utils/helpers.h"

class CarState { 
    public:
        CarState(double prev_car_x, double prev_car_y, double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed) 
                : prev_car_x(prev_car_x), prev_car_y(prev_car_y), car_x(car_x), car_y(car_y), car_s(car_s), car_d(car_d), car_yaw(car_yaw), car_speed(car_speed) {}

        double prevCarX() const { return prev_car_x; }

        double prevCarY() const { return prev_car_y; }

        double carX() const { return car_x; }

        double carY() const { return car_y; }

        double carS() const { return car_s; }

        double carYaw() const { return car_yaw; }

        double carSpeed() const { return car_speed; }

        double carD() const { return car_d; }        

        int carLane() const { return deriveLane(car_d); }

    private:
        double prev_car_x; 
        double prev_car_y;
        double car_x;
        double car_y;
        double car_s;
        double car_d;
        double car_yaw;
        double car_speed;
};

#endif