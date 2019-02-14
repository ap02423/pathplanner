#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/LU"



#include "helpers.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


using Eigen::MatrixXd;
using Eigen::VectorXd;

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
		  //auto prev_path_x = j[1][prev_path_s"]
		  
		  
          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

		
		
		int index;
		
		double a = car_s;
		double b = car_d;
		double c = car_speed;
		double target_velocity = 25.00;
		vector<double> xy;
		
	
		if (previous_path_x.size() == 0)
			
			{
				vector<double> result;
				vector <double> start;
				vector <double> end;
				double T= 4.0; 
				
				start.push_back(a);
				start.push_back(c);
				start.push_back(0);
				
				
				end.push_back(a+target_velocity/2.237 * T);
				end.push_back(target_velocity/2.237);
				end.push_back(0);
				
				result = JMT( start,  end, T);
				
				
				//0.02 seconds is when simulator updates
				
				for (index=0; index < T/0.02; index++)//20
				{
				
				
				double sum=0.0;
				
				//apply the co-efficients to determine next position
				
				for (int i = 0; i < 6 ; i++)
				{
					
					sum+= (pow(0.02*(index+1),i)  * result[i]);
					
				}
				
				a = sum;
				b = car_d;
				xy=getXY(a, b, map_waypoints_s, map_waypoints_x, map_waypoints_y);
				next_x_vals.push_back(xy[0]);
				next_y_vals.push_back(xy[1]);
				}
				 
			}
			else
			{
				
				 
				
				for (index = 0; index < previous_path_x.size();index++)
				{
					next_x_vals.push_back(previous_path_x[index]);
					next_y_vals.push_back(previous_path_y[index]);
				}
				
				
				int size = previous_path_x.size();
				vector<double> result;
				vector <double> start;
				vector <double> end;
				double T= 4.0; //Typically simulator consumes 3 points before next trajectoryis created 
				
				a = end_path_s;
				
				start.push_back(a);
				start.push_back(target_velocity/2.237);
				start.push_back(0);
				
				end.push_back(a+target_velocity/2.237 * T);
				end.push_back(target_velocity/2.237);
				end.push_back(0);
				
				result = JMT( start,  end, T);
				
				
				for (index=0; index < T/0.02 - previous_path_x.size(); index++)
				{
			
				double sum=0.0;
				
				for (int i = 0; i < 6 ; i++)
				{
					
					sum+= (pow(0.02*(index+1),i)  * result[i]);
					
				}
				
				a = sum;
				b = car_d;
			




				xy=getXY(a, b, map_waypoints_s, map_waypoints_x, map_waypoints_y);
				next_x_vals.push_back(xy[0]);
				next_y_vals.push_back(xy[1]);
							
				
				
				
				
				}
				
			//	std::cout << "size" << next_x_vals.size() <<  " " << previous_path_x.size() << std::endl;
			//	std::cout << "size" << 
				
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


