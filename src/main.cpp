#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

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

  //Start in lane 1
  int lane = 1;

  //Target velocity
  //double ref_vel = 49.5; //mph
  double ref_vel = 0.0; //mph


  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          	//auto sensor_fusion = j[1]["sensor_fusion"];
          	vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
          	bool car_ahead = false;
          	bool car_to_my_left = false;
          	bool car_to_my_right = false;
          	double diff_in_speed = 0.0;

          	int prev_size = previous_path_x.size();
          	bool too_close = false;

          	if (prev_size > 0){
          		car_s = end_path_s;
          	}


          	for(int i = 0;i<sensor_fusion.size();i++){
          		//check if the car is in our lane
          		float d = sensor_fusion[i][6];
          		int other_car_lane = -1;
          		if(d>0 && d<4){
          			other_car_lane = 0;
          		}
          		else if(d>4 && d<8){
          			other_car_lane = 1;
          		}
          		else if(d>8 && d<12){
          			other_car_lane = 2;
          		}
          		if(other_car_lane < 0){
          			continue;
          		}


          		
      			double vx = sensor_fusion[i][3];
      			double vy = sensor_fusion[i][4];
      			double check_speed = sqrt(vx*vx + vy*vy);
      			double check_car_s = sensor_fusion[i][5];

      			check_car_s += ((double)prev_size*0.02*check_speed);

          			/*if((check_car_s>car_s) && ((check_car_s-car_s) < 30)){
          				//ref_vel = 29.5;
          				too_close = true;
          				if(lane > 0){
          					lane = 0;
          				}
          			}*/
      			if(other_car_lane == lane){
      				//Same lane
      				car_ahead |= (check_car_s>car_s) && ((check_car_s-car_s) < 30);
      			}
      			else if(other_car_lane - lane == -1){
      				//Other car is left to us
      				car_to_my_left |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
      			}
      			else if(other_car_lane - lane == 1){
      				car_to_my_right |= car_s - 30 < check_car_s && car_s + 30 > check_car_s; 
      			}
          		
          	}

          	if (car_ahead){
          		if(!car_to_my_left && lane>0){
          			//No car to left and we are in either lane 1 or 2
          			lane = lane - 1; //go to left lane
          		}
          		else if(!car_to_my_right && lane!=2){
          			//No car to right and lane is either 0 or 1
          			lane = lane + 1;
          		}
          		else{
          			diff_in_speed -= 0.224;

          		}
          	}
          		else{
          			if(lane != 1){
          				if((lane==0 && !car_to_my_right) || (lane==2 && !car_to_my_left)){
          					lane = 1; //center
          				}
          			}
          			if(ref_vel < 49.5){
          				diff_in_speed += 0.224;
          			}

          		}


          	//vector<double> next_x_vals;
          	//vector<double> next_y_vals;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
/*          	double dist_inc = 0.5;
		    for(int i = 0; i < 50; i++)
		    {
		    	//1. Take next_s and next_d and then find x and y equivalent so that the car remains in the same lane and in center.

		    	double next_s = car_s + (dist_inc*(i+1));
		    	double next_d = 6; //Distance from yellow line which is center of the road to current pos (4(width of lane 1) + 2(center of lane 2))
		    	std::vector<double> x_y = getXY(next_s,next_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);

		        next_x_vals.push_back(x_y[0]);
		        next_y_vals.push_back(x_y[1]);
		    }
*/

          		//2. After 1, we have many warnings like accel, jerk, collision etc. GetXY functions works well except boudary cases. Map is made up of
          		//sparse waypoints and we have corners and line segments. At corners, these points are bunched up which causes accel and jerk
          		//We are violating the speed limit. One of the reason is dist_inc = 0.5. If we reduce it to say 0.3 it will help us to reduce only
          		//average speed but there will still be time on those corner cases where the points will bunch up and we will have jerk.
          		//We need to smooth out the path. We have to use a polynomial that fit through waypoints. Spline is guaranteed to go through all the points
          		//because it is piecewise function of polynomials. 

          		// We have to use spline.h to smoothing out disjointed path that car is following in simulator till now.

          	//Widely spaced (x,y) waypoints, evenly spaced at 30 m. 
          	vector<double> ptsx;
          	vector<double> ptsy;

          	//Reference x,y,yaw states.
          	//It will be either starting point or previous state end point.
          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);

          	//Condition where we do not have previous points. We are just starting.
          	if(prev_size < 2){
          		//Use two points that make the path tangent to the car
          		double prev_car_x = car_x - cos(car_yaw);
          		double prev_car_y = car_y - sin(car_yaw);

          		ptsx.push_back(prev_car_x);
          		ptsx.push_back(car_x);

				ptsy.push_back(prev_car_y);
          		ptsy.push_back(car_y);

          	}
          	//We have previous points. And basically what we are doing here is that we are looking at what were
          	//the last couple of points in the previous path that the car was following. And then calculating at
          	//what angle(heading) the car was moving using last couple of points. And then we are pushing them 
          	//on previous point list.
          	else{
          		//Previous will be the new reference points
          		ref_x = previous_path_x[prev_size - 1];
          		ref_y = previous_path_y[prev_size - 1];

          		//second last previous point
          		double ref_x_prev = previous_path_x[prev_size - 2];
          		double ref_y_prev = previous_path_y[prev_size - 2];
          		ref_yaw = atan2(ref_y - ref_y_prev,ref_x - ref_x_prev);

          		//Use two points (as above in if) that makes the path tangent to the previous path's end point
          		ptsx.push_back(ref_x_prev);
          		ptsx.push_back(ref_x);

          		ptsy.push_back(ref_y_prev);
          		ptsy.push_back(ref_y);
          	}

          	//Now instead of creating 50 points like before, we are creating some waypoints which are sparse
          	//to each other (30 m far) in Frenet coordinate. Instead of 0.5m now we have 30 m. Only three points.

          	vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          	vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          	vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          	ptsx.push_back(next_wp0[0]);
          	ptsx.push_back(next_wp1[0]);
          	ptsx.push_back(next_wp2[0]);


          	ptsy.push_back(next_wp0[1]);
          	ptsy.push_back(next_wp1[1]);
          	ptsy.push_back(next_wp2[1]);

          	//By now the ptsx and ptsy have two previous points and plus they have the location of car at 30m , 60m and 90m. FIVE points.

          	//Now do shift in rotation transofrmation just as MPC. Basically, shifting car's current position to (0,0) and heading is also zero.
          	//Makes life easier while doing maths.

          	for(int i=0;i<ptsx.size();i++){
          		double shift_x = ptsx[i] - ref_x;
          		double shift_y = ptsy[i] - ref_y;

          		ptsx[i] = (shift_x *cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw));
          		ptsy[i] = (shift_x *sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw));
          	}


          	//Now use spline to create curve which joins these points(Points which are in ptsx/ptsy vectors)

          	tk::spline s;

          	//set x,y points to spline
          	s.set_points(ptsx,ptsy);
          	std::vector<double> next_x_vals;
          	std::vector<double> next_y_vals;


          

          	for (int i = 0; i < previous_path_x.size(); i++)
          	{
          		
          		next_x_vals.push_back(previous_path_x[i]);
          		next_y_vals.push_back(previous_path_y[i]);


          	}



          	//Calculate how to break up spline points so that we travel at our desired reference velocity.
          	/* So we have sparse points in spline variable. we need to find intermidiate points from let's say
          	   0-30m. Now we have a spline curve from 0-30 m. Imagine a car at 0,0 and heading in the direction
          	   of spline curve. Imagine a triangle. x = 30m and a spline curve path. Linearize the curve.
          	   Now on hypotenuse,we have to find the N points according to the velocity. We know that the car
          	   will be at next point in 0.02 seconds and if the hypotenuse is distance d (known) then N can be
          	   found as N * 0.02 * velocity = d */
          	//This is what we are doing below.
          	//Here we have x = 30m in target_x variable. we are getting y coordinate targey_y by passing target_x
          	//to the spline.
          	//Calcualting d in target_dist variable.
          	//Inside the following loop, calculating N.

         	double target_x = 30.0;
          	double target_y = s(target_x);
          	double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

          	double x_add_on = 0;

          	//We will always have 50 points all the time. so following loop is run for 50 - previous_path_x.size
          	//In a PACMAN analogy, if the car has eaten 3 points then we will have 47 points and in that case, we
          	//just have to get three more points.
          	for (int i = 1; i <= 50 - previous_path_x.size(); i++)
          	{
          		ref_vel += diff_in_speed;
          		if(ref_vel > 49.5){
          			ref_vel = 49.5;
          		}
          		else if(ref_vel < 0.224){
          			ref_vel = 0.224;
          		}
          		double N = (target_dist/(0.02*ref_vel/2.24));
          		double x_point = x_add_on + (target_x)/N;
          		double y_point = s(x_point);
          		//we have a (x_point,y_point) pair now.

          		//Make current x_point as add on point.
          		x_add_on = x_point;

          		double x_ref = x_point;
          		double y_ref = y_point;

          		//rotate back to normal after rotating it to earlier
          		//Local coordinates to global coordinate system.
          		x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
          		y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

          		x_point += ref_x;
          		y_point += ref_y;

          		next_x_vals.push_back(x_point);
          		next_y_vals.push_back(y_point);
          		std::cout << "Here" << std::endl;
          		
          	}

          	//So if we have points from previous path, the logic is to add them to the path planner
          	//This helps in transition. Here we are making sure that rather than re-creating the path
          	//again from scratch, why not just add points to that and work with what you alredy had
          	//from last time.

          	/*NOTE : We are dealing with two sets of points. One is sparsed(far) waypoints which are in
          			 spline variable s and other is path planning points which are in next_x_vals/y_vals*/

          	for (std::vector<double>::const_iterator i = next_x_vals.begin(); i != next_x_vals.end(); ++i)
    		{
    			std::cout << *i << ' ';
    		}

          	json msgJson;
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
