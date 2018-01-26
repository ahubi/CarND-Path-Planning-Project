#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <cstdio>
#include <vector>
#include <set>
#include <algorithm>
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

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
  //operator to sort objects, required by std containers
  bool operator<(const car_obj &r){return dist2me < r.dist2me;};
  car_obj(){};
  car_obj(const double s_,
          const float d_,
          const double v_,
          const double d2m)
          : s(s_)
          , d(d_)
          , v(v_)
          , dist2me(d2m)
          {};
};

class path_planer {
private:
  unsigned long cycle_count_;               //counts update cycles
  unsigned long lane_change_cycle_;         //at which cycle was lane change
  int safe_distance_front;                  //distance to car required for safe lane change
  int safe_distance_back;                   //distance to back car required for safe lane change
  double my_speed;                          //speed of the self driving car
  vector<vector<car_obj>> lane_obj_front_;   //keeps objects in front of me
  vector<vector<car_obj>> lane_obj_back_;    //keeps objects behind me
  //finds next free lane
  int get_next_free_lane(const int& lane);
  //checks whether it's safe to change to the lane with passed objects
  bool is_safe_to_chage(vector<car_obj>& front, vector<car_obj>& back);
public:
  virtual ~path_planer ();
  path_planer (const int& safe_distance_front, const int& safe_distance_back);
  //Returns a vector  with {too_close, max_speed, new_lane}
  vector<double> get_next_actions(const int& my_lane, const json& sensor_data);
};
#endif
