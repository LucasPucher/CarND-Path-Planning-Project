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


#define CYCLE				0.02	// Cycle time in seconds
#define LANE_WIDTH	4.0
#define LEFT_LANE		0  
#define MID_LANE		1
#define	RIGHT_LANE	2

// FSM States
#define	START_STATE		0
#define	DRIVE_AHEAD		1
#define DRIVE_FOLLOW	2
#define	CHANGE_LEFT		3
#define CHANGE_RIGHT	4

int vehicle_state = START_STATE;
int	sm_entry = 1;

int	object_ahead_id;
double object_ahead_distance;
double object_ahead_speed;

 

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int st_dbg = 0;
double speed_limit = 26.82; // in m/s
double ref_speed = 26.0;
double speed = 0;
int	lane_id = MID_LANE;


#ifdef DEBUG_ACTIVE
std::ofstream outfile ("./trajectories.txt");
#endif

int g_cycle = 0;

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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
					double next_d = (lane_id * LANE_WIDTH) + LANE_WIDTH / 2.0;
					double car_speed_ms = mph2ms(car_speed);

					// PREDICTION
					int object_ahead 	= 0;
					int object_left 	= 0;
					int object_right 	= 0;
					
					object_ahead_distance = 10000; // very large number
					object_ahead_speed = 0;
					for ( int i = 0; i < sensor_fusion.size(); i++ ) 
					{

						int object_lane = -1;
						double object_id = sensor_fusion[i][0];
						double object_vx = sensor_fusion[i][3];
						double object_vy = sensor_fusion[i][4];
						double object_s = sensor_fusion[i][5];
						double object_d = sensor_fusion[i][6];
						double object_speed = sqrt(object_vx*object_vx + object_vy*object_vy);
						
						//std::cout << sensor_fusion[i][0]<< ":" << "##" << object_s << "##" << object_d << "##" << std::endl;
						
						if((object_d > 0) && (object_d < 4))
						{
							object_lane = LEFT_LANE;
						}
						else if ((object_d > 4) && (object_d < 8))
						{
							object_lane = MID_LANE;
						}
						else if((object_d > 8) && (object_d < 12))
						{
							object_lane = RIGHT_LANE;
						}
						else
						{
							continue;
						}
						
						
						if( (object_lane == lane_id) && (object_s >= car_s) && ( object_s < (car_s + 40) ) )
						{
							object_ahead = 1;
							if( (object_s - car_s) < object_ahead_distance )
							{
								object_ahead_distance = object_s - car_s;
								object_ahead_speed = object_speed;
								object_ahead_id = object_id;
							}
						}
						
						// Check for objects on the left lane
						if( lane_id != LEFT_LANE)
						{
							// if object detected on right lane
							if(object_lane == (lane_id - 1))
							{
								// Calculate trajectory for object
								double dt = 0;
								for(int i = 0; i < 60; i++)
								{
									 double object_s_tr = object_s + object_speed * dt;
									 double car_s_tr = car_s + car_speed_ms * dt;
									 
									 if( (object_s_tr > (car_s_tr - 10.0)) && (object_s_tr < (car_s_tr + 40)))
									 {
											object_left = 1;
											break;
									 }
									 dt = dt + 0.2;
								}
							}
						}
						// Check for objects on the right lane
						if( lane_id != RIGHT_LANE)
						{
							// if object detected on right lane
							if(object_lane == (lane_id + 1))
							{
								// Calculate trajectory for object
								double dt = 0;
								for(int i = 0; i < 60; i++)
								{
									 double object_s_tr = object_s + object_speed * dt;
									 double car_s_tr = car_s + car_speed_ms * dt;
									 
									 if( (object_s_tr > (car_s_tr - 10.0)) && (object_s_tr < (car_s_tr + 40)))
									 {
											object_right = 1;
											break;
									 }
									 dt = dt + 0.2;
								}
							}
						}
					}
					// END PREDICTION
					

					// BEHAVIOR
					
					switch(vehicle_state)
					{
						case START_STATE:
						//speed = speed + 0.15;
						ref_speed = mph2ms(47.0);
						vehicle_state = DRIVE_AHEAD;
						sm_entry = 1;
						break;
						
						case DRIVE_AHEAD:
						// entry action
						if(sm_entry == 1)
						{
							std::cout << "Driving ahead" << std::endl;
							sm_entry = 0;
						}
						
						ref_speed = mph2ms(47.0);
						if( object_ahead == 1 )
						{
							vehicle_state = DRIVE_FOLLOW;
							sm_entry = 1;
						}				
						break;
						
						//Cost to keep lane
						case DRIVE_FOLLOW:
						// entry action
						if(sm_entry == 1)
						{
							std::cout << "Following car " << object_ahead_id << " with distance " << object_ahead_distance;
							std::cout << " [m] and speed " << object_ahead_speed << " [m/s]" << std::endl;
							sm_entry = 0;
						}
						std::cout << "Following car " << object_ahead_id << " with distance " << object_ahead_distance;
						std::cout << " [m] and speed " << object_ahead_speed << " [m/s]" << std::endl;
						ref_speed = object_ahead_speed - 2;
						if( object_ahead == 0 )
						{
							vehicle_state = DRIVE_AHEAD;
							sm_entry = 1;
						}
						else if((speed > ref_speed/2) && (object_left == 0) && (lane_id != LEFT_LANE))
						{
							vehicle_state = CHANGE_LEFT;
							sm_entry = 1;
						}
						else if((speed > ref_speed/2) && (object_right == 0) && (lane_id != RIGHT_LANE))
						{
							vehicle_state = CHANGE_RIGHT;
							sm_entry = 1;
						}					
						break;
						
						case CHANGE_LEFT:
						if(sm_entry == 1)
						{
							std::cout << "Changing lane from " << lane_id << "to " << lane_id - 1 << std::endl;
							sm_entry = 0;
						}						
						lane_id--;
						vehicle_state = DRIVE_AHEAD;
						sm_entry = 1;
						break;
						
						case CHANGE_RIGHT:
						if(sm_entry == 1)
						{
							std::cout << "Changing lane from " << lane_id << "to " << lane_id + 1 << std::endl;
							sm_entry = 0;
						}							
						lane_id++;
						vehicle_state = DRIVE_AHEAD;
						sm_entry = 1;
						break;
					}
					//std::cout << "State:" << vehicle_state << std::endl;

					// do not allow changes in speed more than 0.1 m/s to limit jerk and acceleration
					if(speed > ref_speed)
					{
						speed -= 0.25;
					}
					else if (speed < ref_speed)
					{
						speed += 0.14;
					}
					
					// END BEHAVIOR
					
					

					// TRAJECTORY
					
					int path_size = previous_path_x.size();
					
					// fill the path with previous points
					for(int i = 0; i < path_size ; i++)
					{
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}
					
					
					double theta = deg2rad(car_yaw);
					
					vector <double> x_traj_points;
					vector <double> y_traj_points;
					
					vector <double> x_traj_points_t;
					vector <double> y_traj_points_t;
					
					// Create a spline with: start of car, next position, and three more reference points spaced 30m
					if (path_size < 2) 
					{
						x_traj_points.push_back(car_x - cos(theta));
						x_traj_points.push_back(car_x);
						y_traj_points.push_back(car_y - sin(theta));
						y_traj_points.push_back(car_y);
					} 
					else 
					{
						// redefine reference state as previous path and point
						double x1 = previous_path_x[path_size-2];
						double y1 = previous_path_y[path_size-2];
						double x2 = previous_path_x[path_size-1];
						double y2 = previous_path_y[path_size-1];

						theta = atan2(y2-y1, x2-x1);
						// use two points that make the path tangent to the previous 
						// end point
						x_traj_points.push_back(x1);
						x_traj_points.push_back(x2);
						y_traj_points.push_back(y1);
						y_traj_points.push_back(y2);
					}
					
					vector <double> s_anchor_pos{ 50.0, 80.0, 110.0 };
					for(int i = 0; i < s_anchor_pos.size(); i++)
					{
						vector <double> xy = getXY(car_s+s_anchor_pos[i], next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
						x_traj_points.push_back(xy[0]);
						y_traj_points.push_back(xy[1]);
					}
					
					for(int i = 0; i < x_traj_points.size(); i++)
					{
							double xt = (x_traj_points[i] - car_x) * cos(theta) + (y_traj_points[i] - car_y) * sin(theta);
							double yt = - (x_traj_points[i] - car_x) * sin(theta) + (y_traj_points[i] - car_y) * cos(theta);
							x_traj_points_t.push_back(xt);
							y_traj_points_t.push_back(yt);
					}
					tk::spline s;
					s.set_points(x_traj_points_t, y_traj_points_t);
										
					// Calculate the spacing between points
					double y_at_dist = s(90.0);
					//double dist = distance(0.0, 0.0, 90.0, y_at_dist);
					//double num_points = dist / (CYCLE * speed);
					//dist_inc = 90.0 / num_points;
					double dist_inc = CYCLE * speed;
					
					
					double last_x = 0;
					double last_y = 0;
					if(path_size > 0)
					{
						 last_x = previous_path_x[path_size - 1];
						 last_y = previous_path_y[path_size - 1];
						 last_x = (last_x - car_x) * cos(theta) + (last_y - car_y) * sin(theta);
					}
				
					double j = 1.0;
					for(int i = path_size; i < 60; i++)
					{
						double x = last_x + j * dist_inc;
						double y = s(x);
						double xt = x * cos(theta) - y * sin(theta) + car_x;
						double yt = x * sin(theta) + y * cos(theta) + car_y;
						next_x_vals.push_back(xt);
						next_y_vals.push_back(yt);
						j = j + 1.0;
					}
					// END TRAJECTORY
					
					
					//DEBUG
#ifdef DEBUG_ACTIVE
   					outfile << car_x << ";" << car_y << ";" << car_yaw << ";";
  					for (int i = 0; i < next_x_vals.size(); i++)
  					{
  						outfile << next_x_vals[i] << ";" << next_y_vals[i] << ";"; 
  						
  					}
  					outfile << std::endl;
#endif

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
	
#ifdef DEBUG_ACTIVE
	outfile.close();
#endif
}