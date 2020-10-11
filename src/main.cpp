#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h" // cubic spline library from the internet //
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// Defining some initial conditions for the program
int lane = 1; // lane numbering are 0,1,2
int target_lane = -1; // future lane, always trying to keep the right-most lane
double target_speed = 0.0; // vehicle velocity
double safe_dist = 30.0; // minimum distance to the forward vehicle
double target_spacing = 40.0; // future points for trajectory calculation (hence the intervals being 40m, 80m and 120m)
int path_size = 50; // vector containing the polynomial points for trajectory calculation
int prev_half_of_track = -1;
int wait_counter = 0; // avoiding a double lane change and hence a very high lateral acceleration during lane changes

// Defining some constants
const double weight_speed_penalty = 5.0; // penalty for exceeding speed limit against changing lane
const int half_track_s = 3473; // half of the total length of the track
const double speed_lim = 50.0;
const double max_decel = 0.4;
const double max_accel = 0.224;
const double speed_freedriving = 49.5;
const double lane_center_offset = 2; // distance between the lane lines to the lane center
const double lane_width = 4; // width of each lane
const double look_ahead_distance = 80; // how many meters we can look ahead to understand vehicle and traffic behaviour


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          /*Code Beginning*/
          int prev_size = previous_path_x.size();

          int half_of_the_track = (int(car_s) / half_track_s) % 2;

          //DEBUGGING
          if (prev_half_of_track != half_of_the_track) {
              std::cout<<"CURRENTLY" << half_of_the_track << "PART OF THE TRACK" << std::endl;
              prev_half_of_track = half_of_the_track;
              if (half_of_the_track + 1 == 1) {
                  std::cout << "Strategy adopted: Right-most lane possible" << std::endl;
              }
              else if (half_of_the_track + 1 == 2) {
                  std::cout << "Strategy adopted: Lowest cost lane, determined by penalizing lane switching and speed limit penalties" << std::endl;
              }
          }

          // Collision avoidance
          if (prev_size > 0) {
              car_s = end_path_s;
          }

          // Defining variables for trajectory planning
          bool car_in_front = false;
          bool car_on_leftlane = false;
          bool car_on_rightlane = false;

          //Defining variables for speed control
          double dist_car_in_front = 10000;
          double speed_car_in_front = 0;

          // Defining variables for behaviour planning (= lane switching strategy)
          int lane_speed;
          double lane_switch_penalty;
          double speed_penalty;
          double cost;
          int delta_lane = -1;
          int lane_lowest_cost = 1;
          double min_cost = 10000.0;

          vector<double> car_speeds_lane0;
          vector<double> car_speeds_lane1;
          vector<double> car_speeds_lane2;
          vector<double> avg_speeds_lane;
          vector<int> num_cars_lane;

          /*Sensor Fusion Module*/
          for (int i = 0; i < sensor_fusion.size(); i++) {
              // find out if another car is in the same lane as our ego car
              int car_id    = sensor_fusion[i][0];
              float d       = sensor_fusion[i][6];
              double vx     = sensor_fusion[i][3];
              double vy     = sensor_fusion[i][4];
              double check_speed = sqrt(pow(vx, 2) + pow(vy, 2));
              double check_car_s = sensor_fusion[i][5];

              check_car_s += (double)prev_size * 0.02 * check_speed; // Prediction: projecting the car position into the future by using previous points

              double dist2othercar = check_car_s - car_s;
              int lane_other_car = d / 4;
              if (lane_other_car < 0 || lane_other_car>2) {
                  continue;
              }

              // Getting information about average speed and number of cars on each lane
              // Main interest for us being the cars ahead of us in 80m
              if (check_car_s > car_s && dist2othercar > 0 && dist2othercar <= look_ahead_distance) {
                  if (lane_other_car == 0) {
                      car_speeds_lane0.push_back(check_speed); //Left Lane
                  }
                  else if (lane_other_car == 1) {
                      car_speeds_lane1.push_back(check_speed); //Middle Lane
                  }
                  else if (lane_other_car == 2) {
                      car_speeds_lane2.push_back(check_speed); //Right Lane
                  }
              }

              // Setting Flags for Lane changes
              if (lane == lane_other_car) {
                  car_in_front |= check_car_s > car_s && check_car_s - car_s < safe_dist;
                  // Get speed and distance information of the vehicle in front
                  if (check_car_s > car_s && dist2othercar > 0) {
                      speed_car_in_front = check_speed;
                      dist_car_in_front = dist2othercar;
                  }
              }
              else if (lane - lane_other_car == 1) { // Car is on the left lane to us
                  car_on_leftlane |= car_s - safe_dist < check_car_s&& car_s + safe_dist > check_car_s;
              }
              else if (lane - lane_other_car == -1) { // Car is on the right lane to us
                  car_on_rightlane |= car_s - safe_dist<check_car_s&& car_s + safe_dist > check_car_s;
              }
          }

          /* Behavioural Planning*/
          /* Cost Minimisation is based on overall cost being as low as possible for switching lanes and for driving
          below the speed limit. Average speed of each lane needs to be determined*/

          // Find number of cars on each lane
          num_cars_lane.push_back(car_speeds_lane0.size());
          num_cars_lane.push_back(car_speeds_lane1.size());
          num_cars_lane.push_back(car_speeds_lane2.size());

          // Calculating average lane speeds
          avg_speeds_lane.push_back(accumulate(car_speeds_lane0.begin(), car_speeds_lane0.end(), 0.0) / num_cars_lane[0]);
          avg_speeds_lane.push_back(accumulate(car_speeds_lane1.begin(), car_speeds_lane1.end(), 0.0) / num_cars_lane[1]);
          avg_speeds_lane.push_back(accumulate(car_speeds_lane2.begin(), car_speeds_lane2.end(), 0.0) / num_cars_lane[2]);

          for (int i = 0; i < avg_speeds_lane.size(); i++) {
              if (num_cars_lane[i] == 0) {
                  avg_speeds_lane[i] = speed_lim;
              }
              delta_lane = abs(lane - i);
              lane_speed = avg_speeds_lane[i];
              // Evaluate penalty for switching lanes
              lane_switch_penalty = (double)delta_lane * (1 - exp(-delta_lane));
              // Evaluate driving below speed limit
              speed_penalty = (double)abs(speed_lim - lane_speed) / speed_lim;
              // Overall weighted cost sum of both costs (current ratio speed_cost to Lane_switch cost = 5:1)
              cost = lane_switch_penalty + weight_speed_penalty * speed_penalty;
              // store the minimum cost and the lane on which this minimum cost has occured
              if (cost < min_cost) {
                  lane_lowest_cost = i;
                  min_cost = cost;
              }
          }

          // Some other strategies for setting the target lane
          // Least cars
          //int lane_least_cars = std::distance(num_cars_lane.begin(), std::min_element(num_cars_lane.begin(), num_cars_lane.end()));
          //int lane_highest_avgspeed = std::distance(avg_speeds_lane.begin(), std::max_element(avg_speeds_lane.begin(), avg_speeds_lane.end()));
          target_lane = lane_lowest_cost; // Current Setting: Target Lane is the one with the lowest cost

          // Define actions for the two situations: Leading vehicles and cruising
          double speed_increment = 0;

          if (car_in_front) { // Leading vehicle
              if (!car_on_leftlane && lane > 0) {
                  lane--; // Perform Lane Change if there is no car on the left lane
              }
              else if (!car_on_rightlane && lane != 2) {
                  lane++; // Perform Lane Change if there is no car on the right lane
              }

              else {
                  /*Control Speed*/
                  // if no lane change is possible we must slow down -> use a very simple speed controller instead of decelerating by a fixed 0.224 mph per 0.2seconds
                  // calculate required deceleration in mph per second using the speed difference between our ego vehicle and the car directly in front of
                  double diff_speed_mps = ((target_speed / 2.24) - speed_car_in_front);
                  double decel_mphps = diff_speed_mps * 2.24 * 0.02;

                  // avoid jerking acceleration in case of negative speed
                  if (decel_mphps < 0) {
                      decel_mphps = 0.056;
                  }

                  // Avoid collision
                  if (dist_car_in_front < 15) {
                      std::cout << "WARNING: COLLISION. EMERGENCY BRAKING DEPLOYED" << std::endl;
                      decel_mphps = max_decel; // Violation Limit is set to 8 m/s2
                  }
                  // Decelerate as necessary
                  speed_increment = -decel_mphps;
              }
          }
          else { // Cruising
              // Strategy adopted: Right-most lane possible
              if (half_of_the_track == 0) {
                  if ((lane == 0 && !car_on_rightlane)) {
                      lane = 1;
                  }
                  else if ((lane == 1 && !car_on_rightlane)) {
                      lane = 2;
                  }
              }
              // Strategy adopted: Lowest cost lane, determined by penalizing lane switching and speed limit penalties 
              else if ((half_of_the_track == 1)) {
                  if (lane != target_lane) {
                      //Set a timer for waiting until conducting double lane changes. Countermeasure against max. jerk and acceleration violations.
                      wait_counter++;
                      if (wait_counter > 25) { // Wait set: 25 samples * 0.02s = 0.5 secs for lane change
                          if (lane > target_lane && !car_on_leftlane) {
                              std::cout << "Changing Lanes from: " << lane;
                              lane--;
                              std::cout << " to " << lane << std::endl;
                          }
                          else if (lane < target_lane && !car_on_rightlane) {
                              std::cout << "Changing Lanes from: " << lane;
                              lane++;
                              std::cout << " to " << lane << std::endl;
                          }
                          wait_counter = 0;
                      }
                  }
              }

              if (target_speed < speed_freedriving) {
                  speed_increment = max_accel;
              }
          }

          /*Code Ending*/

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /*Trajectory Generation*/

          vector<double> ptsx;
          vector<double> ptsy;

          // Identifying reference vehicle points
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = car_yaw;

          // If previous path is empty, then use the state reference of the car
          if (prev_size < 2) {
              //Generate two points to make path thats tangent to the cars state
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x); //first point
              ptsx.push_back(car_x); // second point

              ptsy.push_back(prev_car_y); //first point
              ptsy.push_back(car_y); // second point
          }

          else { //if possible to build on previous path
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double prev_ref_x = previous_path_x[prev_size - 2];
              double prev_ref_y = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

              ptsx.push_back(prev_ref_x);
              ptsx.push_back(ref_x);

              ptsy.push_back(prev_ref_y);
              ptsy.push_back(ref_y);
          }

          // create evenly spaced points e.g. 30m apart starting from reference points (can be defined using variable int apart = 30)
          vector<double> next_wp0 = getXY(car_s + target_spacing * 1, lane_center_offset + lane_width * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + target_spacing * 2, lane_center_offset + lane_width * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + target_spacing * 3, lane_center_offset + lane_width * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          //Step 1: Transformation into the car coordinates
          for (int i = 0; i < ptsx.size(); i++) {
              // translation
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              // rotation
              ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
              ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
          }

          //Step 2: Fitting points by creating spline
          tk::spline s;
          s.set_points(ptsx, ptsy);

          //Step 3: Build new parthh by starting with previous path
          for (int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }

          //Step 4: Spacing points of generated spline in order to keep desired speed
          double target_x = target_spacing;
          double target_y = s(target_x); // Y for a given X according to the spline function
          double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

          double x_add_on = 0.0;

          //Step 5: Add points of the Spline to the new path to full up remaining points of the path
          for (int i = 0; i < path_size - previous_path_x.size(); i++) {
              target_speed += speed_increment;
              if (target_speed > speed_freedriving) {
                  target_speed = speed_freedriving;
              }
              else if(target_speed<max_accel){
                  target_speed = max_accel;
              }
              double N = target_dist / (0.02 * target_speed / 2.24); // Number of points for splitting up the trajectory along the target distance; converting from mph to m/sec.
              double x_point = x_add_on + target_x / N;
              double y_point = s(x_point);

              x_add_on = x_point;
              double x_ref = x_point;
              double y_ref = y_point;

              // transform back to global coordinate from car coordinates
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}