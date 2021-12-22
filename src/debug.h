#include <vector>
#include <string>
#include <iostream>

#include "TransitionState.hpp"

using std::string;
using std::vector;
using std::cout;

inline void print_coords(string description, const vector<double> &x,const vector<double> &y) {
    cout << description << ": ";

    for (int i = 0; i < x.size(); i++) {
        cout << "(" << x[i] << ", " << y[i] << ") ";
    }

    cout << "\n";
}

inline void print_state(string description, TransitionState &s) {
    cout << "[" << description << "] Lane: " << s.getLane() << " Speed: " << s.targetSpeed() << "\n";
}