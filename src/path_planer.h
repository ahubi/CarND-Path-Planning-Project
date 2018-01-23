#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <cstdio>
#include <vector>
#include <set>
#include <algorithm>
#include "json.hpp"

using json = nlohmann::json;
using namespace std;
//d value of a lane
inline float dOf(int lane){return 2+4*lane;}

//returns lane which corresponds to objects d frenet coordinate
inline int get_obj_lane(const double& d){
  if(d > 0 && d < 4)
    return 0;
  else if(d > 4 && d < 8)
    return 1;
  else if(d > 8 && d < 12)
    return 2;
  else
    return -1; //invalid lane id
}

struct car_obj{
  double s;         //frenet s coordinate of the object
  float d;          //frenet d coordinate of the object
  double v;         //velocity/speed of the object
  double dist2me;   //distance to me at time of observation
  bool operator<(const car_obj &r){return dist2me < r.dist2me;};
};

class path_planer {
private:

  unsigned long cycle_count_;               //counts update cycles
  unsigned long lane_change_cycle_;         //at which cycle was lane change
  int safe_distance_front;
  int safe_distance_back;
  vector<vector<car_obj>> lane_obj_front_;   //keeps objects in front of me
  vector<vector<car_obj>> lane_obj_back_;    //keeps objects behind me

  int get_next_free_lane(const int& lane);   //finds next free lane
  bool is_safe_to_chage(vector<car_obj>& front,
                        vector<car_obj>& back);
public:
  path_planer (const int& safe_distance_front, const int& safe_distance_back);
  virtual ~path_planer ();
  vector<int> get_next_actions(const int& my_lane,
                               const json& sensor_data);
};
#endif
