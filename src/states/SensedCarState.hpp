#ifndef SENSED_CAR_STATE_H
#define SENSED_CAR_STATE_H

class SensedCarState { 
    public:
        SensedCarState(double car_s, double car_d, double car_speed) 
                : car_s(car_s), car_d(car_d), car_speed(car_speed) {}

        double s() const { return car_s; }

        double speed() const { return car_speed; }

        double d() const { return car_d; }        

    private:
        double car_s;
        double car_d;
        double car_speed;
};

#endif