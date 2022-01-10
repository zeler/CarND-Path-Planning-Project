#include <vector>
#include <string>
#include <iostream>

#include "../states/TransitionState.hpp"

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

inline void print_state_vector(string description, vector<TransitionState> &s) {
    cout << description << ": ";

    for (int i = 0; i < s.size(); i++) {
        cout << "(Lane: " << s[i].getLane() << ", Speed: " << s[i].targetSpeed() << ") ";
    }

     cout << "\n";
}

inline void print_vector(string description, vector<double> v) {
    cout << description << ": (";

    for (int i = 0; i < v.size(); i++) {
        cout << v[i];

        if (i < v.size() - 1) {
             cout << ", ";
        }
    }

    cout << ")\n";
}


    