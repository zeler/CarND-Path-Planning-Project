#ifndef STATE_H
#define STATE_H

class State { 
    public:
        State(int lane, double car_speed, bool deccelerate) : lane(lane), car_speed(car_speed), deccelerate(deccelerate), executed(false) {};

        int getLane() { return lane; }
        double getCarSpeed() { return car_speed; }
        bool isExecuted() { return executed; }
        void setExecuted(bool executed) { this->executed = executed; }
        bool isDeccelerate() { return deccelerate; }

    private:
        int lane;
        double car_speed;
        bool executed;
        bool deccelerate;
};

#endif