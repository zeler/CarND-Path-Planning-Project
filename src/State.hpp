#ifndef STATE_H
#define STATE_H

class State { 
    public:
        State(int lane, double target_speed) : lane(lane), target_speed(target_speed), executed(false) {};

        int getLane() { return lane; }
        double getTargetSpeed() { return target_speed; }
        bool isExecuted() { return executed; }
        void setExecuted(bool executed) { this->executed = executed; }

    private:
        int lane;
        double target_speed;
        bool executed;
};

#endif