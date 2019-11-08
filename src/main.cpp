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
    int car_lane = 1;
    double ref_vel = 0.0;

    h.onMessage([&ref_vel,&car_lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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
            
        // previouse state size
          int prev_size = previous_path_x.size();
        
        

            

          

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
            // prediction (Find if there is vehecle around)
            
            // set car's lane. (Middle = 1)
            
            if (prev_size>0){
                car_s = end_path_s;
            }
            bool too_close = false;
            bool v_ahead = false;
            bool v_right = false;
            bool v_left = false;
            for(int i = 0 ; i< sensor_fusion.size();i++){
                // find which lane the vehicle is
                int v_lane = -1;
                float d = sensor_fusion[i][6];
                if (d >= 0 && d<4){
                    v_lane = 0;
                }else if (d>=4 && d<8){
                    v_lane = 1;
                }else if (d>=8 && d<=12){
                    v_lane = 2;
                }
                if (v_lane < 0){
                    continue;
                }
                // find vehicle speed
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double v = sqrt(vx*vx + vy*vy);
                double current_s = sensor_fusion[i][5];
                double estimated_s = current_s +(double)prev_size * v *0.02;
                
                // check another vehicle is at left, right, or ahead (vechicle located 30 units in s axis)
                if (v_lane == car_lane && (estimated_s > car_s && estimated_s-car_s<5)){
                    too_close = true;
                }
                if (v_lane == car_lane && (estimated_s > car_s && estimated_s-car_s<30)){
                    v_ahead = true;
                }else if (v_lane - car_lane == -1 && ((car_s - 15) < estimated_s && (car_s +30) >  estimated_s)){
                    v_left = true;
                }else if (v_lane - car_lane == 1 && ((car_s - 15) < estimated_s && (car_s +30) > estimated_s)){
                    v_right = true;
                }
            }
            
            // behavior planning
            const double Max_speed = 49.5;
            const double Max_acc = 0.224;
            
            // go at max speed
            // reduce speed if car ahead
            // prepare for lane change
            // change lane
            if (v_ahead==false){
                //if there is no vehicle ahead, drive at max speed
                if (ref_vel < Max_speed){
                    ref_vel += Max_acc;
                }else if (ref_vel >= Max_speed){
                    ref_vel -= Max_acc;
                }
            }else if (v_ahead == true){
                //  vehicle located ahead
                if(too_close){
                    ref_vel -= Max_acc*1.5;
                }else if(ref_vel > (car_speed-2)){
                    // reduce speed until it reaches speed of vechicle ahead
                    ref_vel -= Max_acc*1.35;
                }else if (ref_vel < (car_speed-2)){
                    ref_vel += Max_acc;
                }
                if(!v_left && car_lane !=0){
                    // go left lane if empty
                    car_lane -= 1;
                }else if (!v_right && car_lane !=2){
                    // go right lane if empty
                    car_lane += 1;
                }
            }
             
            // reference state
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            
            //create widely spaced xy waypoints
            vector<double> ptsx;
            vector<double> ptsy;
            
            // if there is almost none previous states, use current car state as reference
            if (prev_size < 2) {
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);
                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);
                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            }else {
                // redefine reference state
                ref_x = previous_path_x[prev_size-1];
                ref_y = previous_path_y[prev_size-1];
                // previous state relative to reference state
                double ref_x_prev =previous_path_x[prev_size-2];
                double ref_y_prev =previous_path_y[prev_size-2];
                ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
                

                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);
                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
            }
            
            const int wp_n = 3;
            const double wp_spacing = 30.0;
            const double lane_width = 4.0;
            
            // Add evenly 30m spaced points ahead of the starting reference
            for (int i =0;i<wp_n;i++){
                vector<double> next_wp = getXY(car_s+(wp_spacing*(i+1)),(lane_width/2.0 +(lane_width*car_lane)),map_waypoints_s,map_waypoints_x,map_waypoints_y);
                ptsx.push_back(next_wp[0]);
                ptsy.push_back(next_wp[1]);
            }
            
            // Transform xy coordinate so that car reference angle is at 0 degrees
            for (int i =0; i< ptsx.size();i++){
                double shift_x = ptsx[i] - ref_x;
                double shift_y = ptsy[i] - ref_y;
                
                ptsx[i]=(shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
                ptsy[i]=(shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
            }
            
            // Create spline
            tk::spline s;
            
            // set(x,y) to the spline
            s.set_points(ptsx,ptsy);
            
            // Define (x,y) points used in path planner
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            
            // Add all the previous path points
            for(int i=0; i<prev_size;i++){
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }
            
            //calculte to break up spline points so that car travels at desired velocity
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x*target_x)+(target_y*target_y));
            
            double x_add_on = 0;
            
            // get number of N points when 30m of distance is travelled by 50mph
            // N * 0.02 * ref_vel = dist
            // 0.02 is chosen as car will visit every points per 0.02 seconds
            // N = dist/(0.02*ref_vel)
            // rel_velocity is set in mph. It needs to be transformed to m/s
            // to change mph to m/s, multiply 0.447
            
            for (int i=1;i < 50-prev_size;i++){
                
                double N = (target_dist/(0.02*ref_vel*0.447));
                double x_point = x_add_on + (target_x/N);
                double y_point = s(x_point);
                
                x_add_on = x_point;
                
                double x_ref = x_point;
                double y_ref = y_point;
                
                // transform back to original coordinate system as we've transformed before
                x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
                y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));
                x_point += ref_x;
                y_point += ref_y;
                
                // push back to next_x_vals and next_y_vals
                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }
            
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
