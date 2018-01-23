#include "path_planer.h"

path_planer::path_planer(const int& sdf,
                         const int& sdb)
: cycle_count_(0),
  lane_change_cycle_(0),
  safe_distance_front(sdf),
  safe_distance_back(sdb),
  lane_obj_front_(vector<vector<car_obj>>(3)),
  lane_obj_back_(vector<vector<car_obj>>(3))
{
}
path_planer::~path_planer()
{}
//checks if it's safe to change to this lane
bool path_planer::is_safe_to_chage(vector<car_obj>& front,
                                  vector<car_obj>& back){
  sort(front.begin(), front.end());
  sort(back.begin(), back.end());
  if (front.size()>0 && back.size()==0)
    return (front[0].dist2me > safe_distance_front);
  else if (front.size()==0 && back.size()>0)
    return (back[0].dist2me > safe_distance_back && back[0].v < my_speed);
  else if (front.size()>0 && back.size()>0)
    return (front[0].dist2me > safe_distance_front &&
            back[0].dist2me > safe_distance_back &&
            back[0].v < my_speed);
  else //front and back are empty - lane is safe
    return true;

}
//finds next free lane
int path_planer::get_next_free_lane(const int& current_lane){
  vector<car_obj> front, back, front2, back2;
  int new_lane = -1;
  switch (current_lane) {
    case 0: //check lane 1 only
    case 2:
      front = lane_obj_front_[1];
      back  = lane_obj_back_[1];
      if (is_safe_to_chage(front, back))
        new_lane = 1;
      break;
    case 1: //check lane 0 and 2
      front = lane_obj_front_[0];
      back  = lane_obj_back_[0];
      if (is_safe_to_chage(front, back))
        new_lane = 0;
      front2 = lane_obj_front_[2];
      back2  = lane_obj_back_[2];
      //check second lane
      if (is_safe_to_chage(front2, back2)){
        /*
          check number of cars in front and choose
          lane with smaller number of cars
        */
        if (front2.size() < front.size())
            new_lane=2;
      }
      break;
    default:
      break;
  }
  return new_lane;
}
vector<double> path_planer::get_next_actions(const int& my_lane,
                                          const json& j)
{
  //Main car's localization Data
  double car_x    = j[1]["x"];
  double car_y    = j[1]["y"];
  double car_s    = j[1]["s"];
  double car_d    = j[1]["d"];
  double car_yaw  = j[1]["yaw"];
  my_speed        = j[1]["speed"]; //set speed lane checks
  // Previous path data given to the Planner
  auto previous_path_x = j[1]["previous_path_x"];
  auto previous_path_y = j[1]["previous_path_y"];
  // Previous path's end s and d values
  double end_path_s = j[1]["end_path_s"];
  double end_path_d = j[1]["end_path_d"];

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = j[1]["sensor_fusion"];

  int prev_size = previous_path_x.size();

  if (prev_size > 0) {
    car_s = end_path_s;
  }

  int too_close = 0;
  double max_speed = 49.5;
  int new_lane  = my_lane;
  //clear objects from previous cycle
  for (auto& lobj:lane_obj_front_)
    lobj.clear();
    //clear objects from previous cycle
  for (auto& lobj:lane_obj_back_)
    lobj.clear();

  // find ref_v to use
  for (size_t i = 0; i < sensor_fusion.size(); i++) {
    int car_id          = sensor_fusion[i][0];
    double vx           = sensor_fusion[i][3];
    double vy           = sensor_fusion[i][4];
    float d             = sensor_fusion[i][6];
    double s            = sensor_fusion[i][5];
    double check_car_s  = sensor_fusion[i][5];
    double check_speed  = sqrt(vx*vx+vy*vy);

    //find s of the checking car
    check_car_s += ((double)prev_size*.02*check_speed);

    // Car is in my lane
    if (get_obj_lane(d)==my_lane) {
      //check car is in front of ego car and gap is smaller than 30m
      if((check_car_s > car_s) && (check_car_s - car_s) < (safe_distance_front-1)){
        max_speed = check_speed;
        too_close = 1;
      }
    }
    else{
      int obj_lane = get_obj_lane(d);
      if(obj_lane != -1){
        car_obj c;
        c.d = d;
        c.v = check_speed;
        c.s = check_car_s;
        if((check_car_s >= car_s)){ //in front of my car
          c.dist2me = check_car_s - car_s;
          lane_obj_front_[obj_lane].push_back(c);
        }
        else{ //behind my car
          c.dist2me = car_s - check_car_s;
          lane_obj_back_[obj_lane].push_back(c);
        }
      }
    }
  }
  cycle_count_++;
  //stay at least for 5 cycles in the same lane to avoid jumpy lane changes
  if((cycle_count_ - lane_change_cycle_) > 5){
    new_lane = get_next_free_lane(my_lane);
    if (new_lane != -1)
      lane_change_cycle_ = cycle_count_;
  }

  /*
  cout<<"--> cycle: " << cycle_count_ << endl;
  cout<<"Lane[0] front: " << lane_obj_front_[0].size() <<" back: " << lane_obj_back_[0].size() << endl;
  cout<<"Lane[1] front: " << lane_obj_front_[1].size() <<" back: " << lane_obj_back_[1].size() << endl;
  cout<<"Lane[2] front: " << lane_obj_front_[2].size() <<" back: " << lane_obj_back_[2].size() << endl;
  */
  return {too_close, max_speed, new_lane};
}
