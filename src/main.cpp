#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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
  
  LaneID current_lane_id{LaneID::kMiddleLane}; //car starts in the middle lane
  double ref_velocity{0.0}; //mph
  const double klane_width{4.0};

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_velocity, &klane_width, &current_lane_id]
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
          
          int prev_size = previous_path_x.size();
          const double gap_distance{30.0};
          const double speed_increment{.224}; //0.1 m/s
          const double max_speed{49.5};
          
          if(prev_size > 0)
          {
            car_s = end_path_s;
          }
          
          bool too_close{false};
          
          // check whether cars are in lane
          
          for (int i{0}; i < sensor_fusion.size(); ++i)
          {
            float d{sensor_fusion[i][6]};
            
            const bool within_lane{d < (2 + 4 * static_cast<double>(current_lane_id) + 2) && d > (2 + 4 * static_cast<double>(current_lane_id) - 2)};
            
            if(within_lane)
            {
              double vx{sensor_fusion[i][3]};
              double vy{sensor_fusion[i][4]};
              double check_speed{sqrt(vx * vx + vy * vy)};
              double check_s{sensor_fusion[i][5]};
              check_s += (static_cast<double>(prev_size) * .02 * check_speed);
              
              const bool potential_collision{(check_s > car_s) && (check_s - car_s < gap_distance)};
              
              if(potential_collision)
              {
                
                too_close = true;
                
              }
              
            }
          }
          
          //Finite state machine
          
          if (too_close)
          {
          
          switch(current_lane_id)
           {
            case LaneID::kLeftLane:
            case LaneID::kRightLane:
              {
                bool middle_lane_change_possible = IsLaneChangePossible(LaneID::kMiddleLane, gap_distance, sensor_fusion, car_s, prev_size);
                current_lane_id = (middle_lane_change_possible ? LaneID::kMiddleLane : current_lane_id);
                break;
              }
              
            case LaneID::kMiddleLane:
             {
              // prioritize changing to the left lane
              bool left_lane_change_possible{IsLaneChangePossible(LaneID::kLeftLane, gap_distance, sensor_fusion, car_s, prev_size)};
              
              if (left_lane_change_possible)
              {
                current_lane_id = LaneID::kLeftLane;
              }
              
              else
              {
                bool right_lane_change_possible{IsLaneChangePossible(LaneID::kRightLane, gap_distance, sensor_fusion, car_s, prev_size)};
                
                if (right_lane_change_possible)
                {
                current_lane_id = LaneID::kRightLane;
                }
              }
              
              break;
             }
              
            default:
              break;
              
           }
          }
          
          if ((too_close) && (ref_velocity > speed_increment))
          {
            ref_velocity -= speed_increment;
          }
          
          else if(ref_velocity < max_speed)
          {
            ref_velocity += speed_increment;
          }


          // trajectory generator
          
          vector<double> ptsx{};
          vector<double> ptsy{};
          
          //keep track of car's position
          
          double ref_x{car_x};
          double ref_y{car_y};
          double ref_yaw{deg2rad(car_yaw)};
          
          if(prev_size < 2) // in case we're starting out with almost no previous path
          {
            double prev_car_x{car_x - cos(car_yaw)};
            double prev_car_y{car_y - sin(car_yaw)};
            
            ptsx.emplace_back(prev_car_x);
            ptsx.emplace_back(car_x);
            
            ptsy.emplace_back(prev_car_y);
            ptsy.emplace_back(car_y);
            
          }
          
          else
          {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            
            double ref_x_prev{previous_path_x[prev_size - 2]};
            double ref_y_prev{previous_path_y[prev_size - 2]};
            
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            // use two points from the previous path that make the new path tangent to the previous path's end point
            
            ptsx.emplace_back(ref_x_prev);
            ptsx.emplace_back(ref_x);
            
            ptsy.emplace_back(ref_y_prev);
            ptsy.emplace_back(ref_y);
            
            
          }
          
          // define waypoints separated by the horizon distance to generate a spline
          
          const double target_x{30.0};
          
          double lateral_distance{(klane_width * static_cast<double>(current_lane_id)) + (0.5 * klane_width)};
          
          vector<vector<double>> way_points(3);
          
          for (int i{0} ; i < way_points.size(); ++i)
          {
          
          way_points[i] = getXY(car_s + (i+1)*target_x, lateral_distance, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.emplace_back(way_points[i][0]);
          ptsy.emplace_back(way_points[i][1]);
       
          }
          
          // transform way-points to car's local coordinates
          
          for (int i{0}; i < ptsx.size(); ++i)
          {
            double shift_x{ptsx[i] - ref_x};
            double shift_y{ptsy[i] - ref_y};
            
            ptsx[i] = (shift_x * cos(- ref_yaw) - shift_y * sin(- ref_yaw));
            ptsy[i] = (shift_x * sin(- ref_yaw) + shift_y * cos(- ref_yaw));
          }
          
          //create spline
          
          tk::spline s;
          s.set_points(ptsx, ptsy);
          
          //Define actual points that would be used by the planner
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          for (int i{0}; i < prev_size; ++i)
          {
            next_x_vals.emplace_back(previous_path_x[i]);
            next_y_vals.emplace_back(previous_path_y[i]);
          }
          
          double target_y{s(target_x)};
          double target_distance{sqrt((target_x * target_x) + (target_y * target_y))};
          double N{target_distance / ((.02 * ref_velocity) / 2.24)};
          double x_add_on{0.0};
          
          //break up the spline points into equally spaced points according to the reference velocity
          
          for (int i{1}; i <= 50 - prev_size; ++i)
          {
            double x_point{x_add_on + (target_x / N)};
            double y_point{s(x_point)};
            
            x_add_on = x_point;
            
            double x_ref{x_point};
            double y_ref{y_point};
            
            // rotate back to map frame
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.emplace_back(x_point);
            next_y_vals.emplace_back(y_point);
            
          }
          
          //end

          json msgJson;
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