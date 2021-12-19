#include <vector>
#include <string>
#include <iostream>

using std::string;
using std::vector;
using std::cout;

void print_coords(string description, const vector<double> &x,const vector<double> &y) {
    cout << description << ": ";

    for (int i = 0; i < x.size(); i++) {
        cout << "(" << x[i] << ", " << y[i] << ") ";
    }

    cout << "\n";
}