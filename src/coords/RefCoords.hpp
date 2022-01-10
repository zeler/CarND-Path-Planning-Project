#ifndef REF_COORDS_H
#define REF_COORDS_H

class RefCoords { 
    public:
        RefCoords() {};

        RefCoords(double ref_x_previous, double ref_y_previous, double ref_x, double ref_y, double ref_yaw, double ref_s, double ref_d) 
         : ref_x_previous(ref_x_previous), ref_y_previous(ref_y_previous), ref_x(ref_x), ref_y(ref_y), ref_yaw(ref_yaw), ref_s(ref_s), ref_d(ref_d) {};

        double refXPrevious() const { return ref_x_previous; }

        double refYPrevious() const { return ref_y_previous; }

        double refX() const { return ref_x; }

        double refY() const { return ref_y; }

        double refYaw() const { return ref_yaw; }

        double refS() const { return ref_s; }

        double refD() const { return ref_d; }

    private:
        double ref_x_previous;
        double ref_y_previous;
        double ref_x;
        double ref_y;
        double ref_yaw;
        double ref_s;
        double ref_d;
};

#endif