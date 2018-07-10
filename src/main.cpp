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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
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

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
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

  int lane_number = 1; // lane number: 0 = left lane, 1 = middle lane, 2 = right lane
  float v_dest = 0; // v_dest is the velocity we want to reach

  h.onMessage([&v_dest, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane_number](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
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
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	int prev_size = previous_path_x.size();
          	bool too_close = false;
          	/***********************************************************************************************************
          	 * User code start
          	 *
          	 * define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	 ***********************************************************************************************************/


          	// setup variables

            vector<double> anchors_x; // x anchor values which will be used to calculate a spline over a long distance
            vector<double> anchors_y; // y anchor values which will be used to calculate a spline over a long distance
            double ref_x = car_x; // store current cars x position, later we use this variable for the last calculated path x point
            double ref_y = car_y; // store current cars x position, later we use this variable for the last calculated path y point
            double ref_yaw = deg2rad(car_yaw); // store current cars angel
            float v_dest_max = 49.5;

            // Check the sensor fusion data

            for(int i = 0; i < sensor_fusion.size(); i++)
            {
              double d = sensor_fusion[i][6u];
              // Is the car on our lane?
              if( ((4*lane_number) < d) && ((4+4*lane_number) >= d) )
              {
                // What is the distance of the car (after our path is passed)
                double vx = sensor_fusion[i][3u];
                double vy = sensor_fusion[i][4u];
                double velocity_car = sqrt(vx*vx + vy*vy);
                double distance = sensor_fusion[i][5u];
                distance = (distance + (0.02 * (double)prev_size * velocity_car / 2.24)) - end_path_s;

                if(distance >= 0.0 && distance <= 30.0) /* caution on wrap after highway is completed */
                {
                  too_close = true;
                  v_dest_max = velocity_car;
                  if(lane_number > 0)
                  {
                    lane_number--;
                  }
                  else
                  {
                    lane_number++;
                  }
                }
              }
            }

            if(too_close && (v_dest > v_dest_max))
            {
              v_dest -= 0.224;
            }
            else if(v_dest < 49.5)
            {
              v_dest += 0.224;
            }
            // What is the velocity of the car

            // Penalty these issues

            // Predict the other cars in the future
            // Give some possible trajectories (faster ones, slower ones, w/wo lane changes, early lane changes, later lane changes
            // apply some cost functions on each trajectory and penalty it with a given weight
            // cost functions: the closer (distance) to other vehicles the higher the costs,
            // get the best trajectory and apply it


            // set the first two anchor points.
            if(prev_size < 2)
            {
              // if there is no path calculated, we use the current and the previous position as an anchor point
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);
              anchors_x.push_back(prev_car_x);
              anchors_x.push_back(ref_x);
              anchors_y.push_back(prev_car_y);
              anchors_y.push_back(ref_y);
            }
            else
            {
              // if there is still a previous path (with more than two update values (x,y) left), append the last calculated points as anchor points
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];
              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
              anchors_x.push_back(ref_x_prev);
              anchors_x.push_back(ref_x);
              anchors_y.push_back(ref_y_prev);
              anchors_y.push_back(ref_y);
            }


            // Setup further anchor points in a distance of 30, 60 and 90 meters
          	vector<double> next_wp0 = getXY(car_s + 50, (2+4*lane_number), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp1 = getXY(car_s + 100, (2+4*lane_number), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp2 = getXY(car_s + 150, (2+4*lane_number), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	anchors_x.push_back(next_wp0[0]);
          	anchors_x.push_back(next_wp1[0]);
          	anchors_x.push_back(next_wp2[0]);
          	anchors_y.push_back(next_wp0[1]);
            anchors_y.push_back(next_wp1[1]);
            anchors_y.push_back(next_wp2[1]);


            // Shift car reference angle to 0 degrees
            for(int i = 0; i < anchors_x.size(); i++ )
            {
              double shift_x = anchors_x[i] - ref_x;
              double shift_y = anchors_y[i] - ref_y;

              // Avoid to get the spline to get vertical. Shift the with the current global map angle to make sure the spline goes in x direction
              anchors_x[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
              anchors_y[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
            }





            vector<double> next_x_vals;
            vector<double> next_y_vals;

            // reusing previous path points avoids jerk by smoothly incept new trajectories step by step to the end of the previously calculated trajectory
            for(int i = 0; i < prev_size; i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // Define a spline with the set up anchor points, using the spline library
            tk::spline s;
            s.set_points(anchors_x, anchors_y);
            // The spline will be linearized within a distance of 30 meters. Depending on the v_dest, the number of path points over this distance will be calculated

            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x)*(target_x)+ (target_y)*(target_y));
            double x_add = 0.0;



            for(int i = 0; i < 50 - prev_size; i++)
            {
                double N = (target_dist / (0.02 * v_dest / 2.24));
                double x_point = x_add + (target_x)/N;
                double y_point = s(x_point);

                x_add = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                // Shift back to the origin global map angle
                x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

                x_point += ref_x;
                y_point += ref_y;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;



          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
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
