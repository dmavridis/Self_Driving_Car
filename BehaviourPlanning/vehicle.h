#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle {

public:

  bool change_lane = false;

  bool speed_update = false;

  int L = 1;

  int vid;

  double pos_x, pos_y, angle, pos_s, pos_d;

  double vx,vy;

  vector<double> next_x_vals, next_y_vals;

  int lane;

  double speed, target_speed;

  int preferred_buffer = 6; // impacts "keep lane" behavior.


  int s;

  int v;

  int a;

  int lanes_available;

  const double acceleration = 0.224; // miles/s/s



  int goal_lane;

  int goal_s;

  string state;

  /**
  * Constructor
  */
  Vehicle(int vid);

  /**
  * Destructor
  */
  virtual ~Vehicle();


  void UpdateState(double pos_x, double pos_y, double angle, double pos_s);

  void UpdateTrajectory(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s,
                                vector<double> previous_path_x, vector<double> previous_path_y);


  void UpdateOtherState(vector<double> state);

  void UpdateSpeed();

  void LaneChange(int lane);

};

#endif
