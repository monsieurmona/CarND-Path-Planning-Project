#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include "Environment.hpp"
#include "PathPlan.hpp"

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
   // string map_file_ = "../data/highway_map.csv";
   string map_file_ = "/home/mona/src/udacity/CarND-Path-Planning-Project-Root/CarND-Path-Planning-Project/data/highway_map.csv";

   // The max s value before wrapping around the track back to 0
   double max_s = 6945.554;

   std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

   string line;
   while (getline(in_map_, line)) {
      std::istringstream iss(line);
      double x;
      double y;
      double s;
      double d_x;
      double d_y;
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
               CarState carState(
                     j[1]["x"],
                     j[1]["y"],
                     j[1]["s"],
                     j[1]["d"],
                     deg2rad(j[1]["yaw"]),
                     CarState::convertMilesPerHourToMetersPerSecond(j[1]["speed"]));

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

               json msgJson;

               constexpr double updateInterval = 0.02;
               constexpr double laneWidth = 4.0;
               constexpr double nLanes = 3;
               Lane lane(laneWidth);

               Environment environment(nLanes, lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
               environment.setEnvironment(sensor_fusion, carState);

               PathPlan pathPlan(previous_path_x,
                                 previous_path_y,
                                 environment,
                                 carState,
                                 updateInterval);

               const Trajectory & trajectory = pathPlan.getEgoTrajectory();
               /**
                * TODO: define a path made up of (x,y) points that the car will visit
                *   sequentially every .02 seconds
                */


               msgJson["next_x"] = trajectory.getPathAxisX();
               msgJson["next_y"] = trajectory.getPathAxisY();

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
