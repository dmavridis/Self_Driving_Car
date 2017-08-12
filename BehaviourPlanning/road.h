#ifndef ROAD_H
#define ROAD_H
#include <math.h>
#include <vector>
#include <string>
#include "vehicle.h"

// using namespace std;

class Road {

public:

  vector<double> map_waypoints_x;

  vector<double> map_waypoints_y;

  vector<double> map_waypoints_s;

  vector<double> map_waypoints_dx;

  vector<double> map_waypoints_dy;

  vector<int> vehicle_ids;

  // map<int, Vehicle> vehicles; // vehicles caprtured by sensor fusion

  int LANE_WIDTH;

  string map_file_;
  /**
  * Constructor
  */
  Road(string map_file_);

  /**
  * Destructor
  */
  virtual ~Road();

  void PopulateTraffic(map<int , vector<double>> sensor_fusion);

//  Vehicle GetVehicle(int vid);

};

#endif
