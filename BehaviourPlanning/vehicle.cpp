#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include "spline.h"
#include "transforms.h"
#include "transforms.cpp"
/**
 * Initializes Vehicle
 */


Vehicle::Vehicle() {
    cout << "I have been created" << endl;
}

Vehicle::~Vehicle() {}

void Vehicle::UpdatePosition(double pos_x, double pos_y, double pos_s, double pos_d, double angle) {
// Updates the position of the car with new measurement data

    this->pos_x = pos_x;
    this->pos_y = pos_y;
    this->angle = angle;
    this->pos_s = pos_s;
    this->pos_d = pos_d;
    }


void Vehicle::UpdateTrajectory(vector<double> next_x_vals, vector<double> next_y_vals, int next_map_point,
                              vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s){
    this->next_x_vals = next_x_vals;
    this->next_y_vals = next_y_vals;

    // Identify the next closest waypoint

    const int MAP_DEPTH = 6;
    vector<double> X_(MAP_DEPTH), Y_(MAP_DEPTH), S_(MAP_DEPTH);  // get next points of the map and create the spline
    vector<double> XY_(2);
    X_[0] = this->pos_x;
    Y_[0] = this->pos_y;
    S_[0] = this->pos_s;

    int path_size = next_x_vals.size();
    int des_d = 6;


    for (int i = 1; i < MAP_DEPTH; i++){
        S_[i] = map_waypoints_s[next_map_point+i];
        XY_ = getXY(S_[i], des_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        X_[i] = XY_[0];
        Y_[i] = XY_[1];
     }

     tk::spline x_from_s;
     tk::spline y_from_s;
     x_from_s.set_points(S_,X_);
     y_from_s.set_points(S_,Y_);

    double pos_s = this->pos_s;
    // Update the path buffer
    for(int i = 0; i < 50 - path_size; i++)
     {
         pos_s += 0.4;

         double x_traj, y_traj;
         x_traj = x_from_s(pos_s);
         y_traj = y_from_s(pos_s);

         this->next_x_vals.push_back(x_traj);
         this->next_y_vals.push_back(y_traj);


     }

}


void Vehicle::RealizeChangeLane(int lane){


}


