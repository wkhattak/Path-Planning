#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++) {
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen) {
			closestLen = dist;
			closestWaypoint = i;
		}
	}
	return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta - heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4) {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
    closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0) {
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef) {
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++) {
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )) {
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}

// Check if a lane change is possible. Uses sensor fusion data to compute position of other cars
bool canChangeLane(vector<vector<double>> sensor_fusion, double car_s, double car_d, double safe_distance, int lane_to_check,int prev_path_size) {
  bool lane_change_ok = true;
  double nearest_distance = 999999999999;
  float other_car_id;
  bool ahead = true;
  
  for (auto s_f: sensor_fusion) {// go through sensor fusion data
    float other_car_d = s_f[6];
    double other_car_vx =  s_f[3];
    double other_car_vy =  s_f[4];
    double other_car_speed = sqrt(other_car_vx*other_car_vx + other_car_vy*other_car_vy);
    double other_car_s = s_f[5];
    int other_car_lane = (int)other_car_d/4;
    double other_car_future_s = other_car_s + (double)prev_path_size * 0.02 * other_car_speed; 
    
    if (other_car_lane == lane_to_check) { // if same lane 
      double dist_diff = other_car_future_s - car_s;
      if (nearest_distance > fabs(dist_diff)) { // find closest non-ego car
        nearest_distance = fabs(dist_diff);
        ahead = ((other_car_future_s - car_s > 0) ? true : false); // find out if non-ego car ahead or behind
        other_car_id = s_f[0];
      }
    }
  }
  
  if (ahead) lane_change_ok = ((nearest_distance > safe_distance) ? true : false); 
  else lane_change_ok = ((nearest_distance > safe_distance/1.50) ? true : false); // if behind then don't need that much space for safe lane change
  cout << "Car: id="<< other_car_id << ", lane=" << lane_to_check << ", distance="  << nearest_distance << (ahead ? " ahead" : " behind") << endl;
  return lane_change_ok;
}

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
  
  // Initial values
  int lane = 1; // centre lane >>> Lanes = 0,1,2 & Lane centres = 2,6,10 (left lane is 0)
  double velocity = 1.0; // start low & slowly build up to SAFE_VELOCITY when possible
  
  // Constants
  const double SAFE_VELOCITY = 49.5;
  const double DISTANCE_AHEAD = 30; // comfortable driving distance in front
  const double SAFE_DISTANCE = 30; // safe driving distance based on future s position
  const double TOO_CLOSE_DISTANCE = 15; // too close driving distance based on future s position
  const double OVERTAKE_DISTANCE = 30; // safe driving distance for overtaking

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
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&SAFE_VELOCITY,&velocity,&DISTANCE_AHEAD,&SAFE_DISTANCE,&OVERTAKE_DISTANCE,&TOO_CLOSE_DISTANCE](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          // A 2D vector of cars and then those cars' [ id, x, y, vx, vy, s, d]: 
          // unique id, x position in map coordinates, y position in map coordinates, 
          // x velocity in m/s, y velocity in m/s, 
          // s position in frenet coordinates, d position in frenet coordinates.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          cout << "************CYCLE START************" << endl;
          int prev_path_size = previous_path_x.size();
          
          
          /***************************************************************************************************************************
          * Find if ego car is close to any non-ego car
          ***************************************************************************************************************************/ 
          if (prev_path_size > 0 ) car_s = end_path_s; // use previous s as current s value if one exists
           
          bool too_close = false;
          double lead_car_speed;
          double future_distance_to_lead_car;
          double future_nearest_distance = 999999999999;
          int other_car_id;
           
          for (auto s_f: sensor_fusion) {// go through sensor fusion data
            double other_car_d = s_f[6];
            int other_car_lane = (int)other_car_d/4;
            if (other_car_lane == lane) { // is the car in our lane?
              double other_car_vx =  s_f[3];
              double other_car_vy =  s_f[4];
              double other_car_speed = sqrt(other_car_vx*other_car_vx + other_car_vy*other_car_vy);
              double other_car_s = s_f[5];

              // Future s of other car after 0.02 seconds
              // Multiplying by prev_path_size gives estimated position of the other vehicle after prev_path_size number of waypoints have been consumed by the simulator
              double other_car_future_s = other_car_s + (double)prev_path_size * 0.02 * other_car_speed;
             
              // Find future distance to lead car in same lane
              future_distance_to_lead_car = other_car_future_s - car_s;

              // Check if other car will be in front of ego car & inside our buffer area
              if (other_car_future_s > car_s && future_distance_to_lead_car < SAFE_DISTANCE) { 
                too_close = true;
                if (future_nearest_distance > future_distance_to_lead_car) {
                 future_nearest_distance = future_distance_to_lead_car;
                 lead_car_speed = other_car_speed;
                 other_car_id = s_f[0];
                }
              }
            }
          }
          

          /***************************************************************************************************************************
          * Take action e.g. slow down to keep following or change lane if possible
          ***************************************************************************************************************************/ 
          if (too_close) {
            cout << "Lead car: id="<< other_car_id << ", lane=" << lane << ", distance="  << future_nearest_distance << endl;
            cout << "Ego car speed: " << velocity << ", lead car speed: " << lead_car_speed << endl;
            if (velocity > lead_car_speed) {
              if (future_nearest_distance < TOO_CLOSE_DISTANCE) { // critically close, so decelrate quicker
                velocity -= (0.224 * 2); // 0.224 is ~5 m/s^2 below than the 10 m/s^s threshold requirement
              }
              else { // not that close, so decelerate normally
                velocity -= 0.224; // ~5 m/s^2 below than the 10 m/s^s threshold requirement
              }
              cout << "Adjusted ego car speed: " << velocity << endl;
            }           
            
            if (lane == 0) { // current lane = left lane
              lane = 1; // try moving to centre lane
              bool lane_change_ok = canChangeLane(sensor_fusion, car_s, car_d, OVERTAKE_DISTANCE, lane, prev_path_size);
              if (lane_change_ok) { 
                cout << "Changing lane to Centre" << endl;
              }
              else {
                cout << "Unable to change lane to Centre" << endl;
                lane = 0; // can't change lane, so stay in lane
              }
            }
            else if (lane == 1) { // current lane = centre lane
              lane = 0; // first try moving to left lane
              bool lane_change_ok = canChangeLane(sensor_fusion, car_s, car_d, OVERTAKE_DISTANCE, lane, prev_path_size);
              if (lane_change_ok) {
                cout << "Changing lane to Left" << endl;
              }
              else {
                cout << "Unable to change lane to Left" << endl;
                lane = 2; // now try right lane
                bool lane_change_ok = canChangeLane(sensor_fusion, car_s, car_d, OVERTAKE_DISTANCE, lane, prev_path_size);
                if (lane_change_ok) {
                  cout << "Changing lane to Right" << endl;
                }
                else {
                  cout << "Unable to change lane to Right" << endl;
                  lane = 1; // can't change lane, so stay in lane
                }
              }
            }
            else { // current lane = right lane
              lane = 1; // try moving to centre lane
              bool lane_change_ok = canChangeLane(sensor_fusion, car_s, car_d, OVERTAKE_DISTANCE, lane, prev_path_size);
              if (lane_change_ok) {
                cout << "Changing lane to Centre" << endl;
              }
              else {
                cout << "Unable to change lane to Centre" << endl;
                lane = 2; // can't change lane, so stay in lane
              }
            }
          }
          else if (velocity < SAFE_VELOCITY) { // no non-ego car within buffer area
            velocity += 0.224; // accelerate 
            cout << "Speed increased as no car within buffer area" << endl;
          }
                              

          /***************************************************************************************************************************
          * Trajectory generation w/o JMT but using spline & method used in the project walkthrough to avoid jerk
          ***************************************************************************************************************************/ 
          
          // Sparse x,y waypoints evenly spaced at DISTANCE_AHEAD metres used as anchors for spline
          // Later on, more wapoints will be generated via spline's interpolation for a smoother path
          vector<double> ptsx;
          vector<double> ptsy;
          
          // Refeence x,y,yaw to be used later
          // These are either set to either current state or previous (if we have prev state)
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if(prev_path_size == 0) { // if the simulator utilised all points in the previous run or the first run
            double prev_car_x = car_x - cos(car_yaw); // calculate previous x
            double prev_car_y = car_y - sin(car_yaw); // calculate previous y
            
            ptsx.push_back(prev_car_x);
            ptsy.push_back(prev_car_y);
            
            ptsx.push_back(car_x);
            ptsy.push_back(car_y);
          }
          else if(prev_path_size == 1) { // only 1 point left from previous run             
            ptsx.push_back(previous_path_x[0]); // get previous x
            ptsy.push_back(previous_path_y[0]); // get previous y
            
            ptsx.push_back(car_x);
            ptsy.push_back(car_y);
          }
          else if(prev_path_size == 2) {// at least 2 previous points available so use prev path's last point as the starting point
            // Update ref with the last point
            ref_x = previous_path_x[prev_path_size-1];  
            ref_y = previous_path_y[prev_path_size-1];
            
            double ref_prev_x = previous_path_x[prev_path_size-2];
            double ref_prev_y = previous_path_y[prev_path_size-2];
            ref_yaw = atan2(ref_y-ref_prev_y, ref_x-ref_prev_x); // find the angle tangent to last point
            
            ptsx.push_back(ref_prev_x);
            ptsy.push_back(ref_prev_y);
            
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);            
          }
          else {// more than 2 previous points available 
            // Update ref with the last point
            ref_x = previous_path_x[prev_path_size-1];  
            ref_y = previous_path_y[prev_path_size-1];
            
            double ref_prev_x = previous_path_x[prev_path_size-2];
            double ref_prev_y = previous_path_y[prev_path_size-2];
            ref_yaw = atan2(ref_y-ref_prev_y, ref_x-ref_prev_x); // find the angle tangent to last point
            
            double ref_prev_prev_x = previous_path_x[prev_path_size-3];
            double ref_prev_prev_y = previous_path_y[prev_path_size-3];
            
            ptsx.push_back(ref_prev_prev_x);
            ptsy.push_back(ref_prev_prev_y);
            
            ptsx.push_back(ref_prev_x);
            ptsy.push_back(ref_prev_y);
            
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);
          }

          // Add 3 points that are DISTANCE_AHEAD metres apart in s,d coord & then get the corresponding x,y pair
          vector<double> next_wp0 = getXY(car_s+DISTANCE_AHEAD, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+DISTANCE_AHEAD*2, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+DISTANCE_AHEAD*3, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsy.push_back(next_wp0[1]);
          
          ptsx.push_back(next_wp1[0]);
          ptsy.push_back(next_wp1[1]);
          
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp2[1]);
          
          // Shift car ref angle to 0 degrees for easy caculations
          for(int i = 0; i < ptsx.size(); i++) { 
            //cout << "Anchor points " << i << ": x=" << ptsx[i] << ", y=" << ptsy[i] << endl;
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
            ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw); 
          }
          
          
          tk::spline s;
          s.set_points(ptsx, ptsy); // specify anchor points
          
          // Start with adding points not utilised last time i.e. the left over points (the simulator doesn't always utilise all the points)
          for(int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Calculate how to break up spline points so that we achieve desired velocity + jerk free trajectory
          double target_x = DISTANCE_AHEAD;
          double target_y = s(target_x); // use spline to get the corresponding y
          double target_dist = sqrt(pow(0-target_x,2) + pow(0-target_y,2));
                    
          double x_add_on = 0; // origin where the car is now but will keep on changing as we add more points
          
          // target_distance = N*0.02*veloctiy OR N = target_distance/(0.02*veloctiy)
          // N is the no. of segments required from origin to target_distance when travelling at velocity
          // 0.02 is used because the simulator picks a point every 0.02 sec
          // 2.24 is the conversion factor for miles/hour to meter/sec
          double N = target_dist/(0.02*velocity/2.24); // N is the total no. of road sections from current location to DISTANCE_AHEAD metres ahead
          double x_increment = target_x/N; // length of each road section in x-axis
          // Add new x,y points to complete 50 points
          for (int i = 1; i <= 50-previous_path_x.size(); i++) { 
            double x_point = x_add_on + x_increment; // start location + length of each road section in x-axis
            double y_point = s(x_point); // find corresponding y 
            
            x_add_on = x_point; // current point becomes the start for next point generation in the loop
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // rotate back to global coord from local coor
            x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
            y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);
            
            x_point += ref_x; // keep the car ahead of the last point
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
            
            //cout << "Newly added point " << i << " coords: x=" << x_point << ", y=" << y_point << endl;
          }

          cout << "************CYCLE END**************" << endl;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } 
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}