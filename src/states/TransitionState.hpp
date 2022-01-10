#ifndef TRANSITION_STATE_H
#define TRANSITION_STATE_H

class TransitionState { 
    public:
        TransitionState(int lane, double target_speed) : lane(lane), target_speed(target_speed) {}
        
        int getLane() const { return lane; }
        double targetSpeed() const { return target_speed; }
    
    private:
        int lane;
        double target_speed;
};

#endif