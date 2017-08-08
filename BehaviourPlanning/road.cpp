#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include "road.h"
#include "vehicle.h"

Road::Road(string map_file_) {


    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;
    double lane_width = 4;

    ifstream in_map_(map_file_.c_str(), ifstream::in);
    string line;
    while (getline(in_map_, line)) {
          istringstream iss(line);
          double x;
          double y;
          float s;
          float d_x;
          float d_y;

          iss >> x;
          iss >> y;
          iss >> s;
          iss >> d_x;
          iss >> d_y;
          this->map_waypoints_x.push_back(x);
          this->map_waypoints_y.push_back(y);
          this->map_waypoints_s.push_back(s);
          this->map_waypoints_dx.push_back(d_x);
          this->map_waypoints_dy.push_back(d_y);
    }
    cout << "Created Landascape" << endl;
}


//Vehicle Road::GetVehicle(int vid){
//    // This excludes ego vehicle that has an id of 0
//    Vehicle TempVehicle = Vehicle(vid);

//    return TempVehicle;
//}

void Road::PopulateTraffic(map<int , vector<double>> sensor_fusion){
    // updates the traffic from the sensor fusion data

    map<int, vector<double>>::iterator it = sensor_fusion.begin();

    while(it != sensor_fusion.end()){
        // check if already exists, if not create vehicle
        if(find(this->vehicle_ids.begin(), this->vehicle_ids.end(), it->first) != this->vehicle_ids.end()){
            this->vehicle_ids.push_back(it->first);
            this->vehicles.insert(std::pair<int,Vehicle>(it->first, Vehicle(it->first)));
        }
        // Append state
        this->vehicles.at(it->first).UpdateState(it->second);
        it++;
    }

}

Road::~Road() {}





