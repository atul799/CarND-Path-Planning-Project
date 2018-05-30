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

//double lane_width=4.0;

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

	//start in lane 1, middle lane
	int lane=1;

	//ref target

	//when ref_vel is not controlled by too_close flag
	//double ref_vel=49.5; //mph
	double ref_vel=0.0;

	//target max vel
	double target_vel=49.5; //mph
	//lane width
	double lane_width=4.0;

	//number of points generated by planner
	int path_length=50;


	h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel,&lane_width,&path_length,&target_vel ](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

					// Sensor Fusion Data, a list of all other cars (on the same side of the road??)
					auto sensor_fusion = j[1]["sensor_fusion"];

					//length of previous path, will be helpful in transition
					int prev_size=previous_path_x.size();

					//sort the sensor fusion data

					//use the end of path s as car_s
					if(prev_size > 0) {

						car_s=end_path_s;
					}

					//flags to set for vehicles ahead in same lane or adjacent lanes
					bool too_close=false;
					bool left_lane_car=false;
					bool right_lane_car =false;
					double car_width=3.0;

					//find ref_v to use
					//sensor fusion has data about all the cars
					//go through each car's data and find out if it is within lane 'lane'
					//of our car
					for (int i=0;i<sensor_fusion.size();i++){

						//check car's lane on d value
						float d=sensor_fusion[i][6];

						//////////////////////////////////
						//find lane of the car the sensor fusion data is from
						int this_car_lane = -1;

						if ( d > 0 && d < lane_width ) {
							this_car_lane = 0;
						} else if ( d > lane_width && d < 2*lane_width ) {
							this_car_lane = 1;
						} else if ( d > 2*lane_width && d < 3*lane_width ) {
							this_car_lane = 2;
						}
						if (this_car_lane < 0) {
							continue;
						}
						//////////////////////////////////
						//capture speed of this car
						//and also set where this car will be
						//after the path has completed trajectory, assuming it goes straight at the current speed
						double vx=sensor_fusion[i][3];
						double vy=sensor_fusion[i][4];

						//check_speed is to calculate predicted distance at end of planned path
						double check_speed=sqrt(vx*vx+vy*vy);

						//s value of the car
						double check_car_s=sensor_fusion[i][5];

						//if using prev points can project s value out
						//if we are using previous points we need to project as we are not there yet
						//looking at where car will be in future

						check_car_s += (double)prev_size*0.02*check_speed;


						//check lane of the car as well as if this car is + 30 /-15 metres from our car
						//also when chaning lane d of our car and d of car has a buffer of 1 metre (car_width/2)
						if ( this_car_lane == lane ) {
							// this vehicle is in our lane
							//check car in front is < 30 m in front
							too_close |= check_car_s > car_s && check_car_s - car_s < 30;
							//and closer than 15 m behind
							//too_close |= car_s - 15 < check_car_s && car_s + 30 > check_car_s;

						} else if ( this_car_lane - lane == -1 ) {
							// this vehicle is in left lane
							//set status available when no car behind 15 m and 30 m front
							left_lane_car |= car_s - 15 < check_car_s && car_s + 30 > check_car_s;
							//left_lane_car |= car_s - 15 < check_car_s && car_s + 30 > check_car_s && abs(car_d-d) > car_width/2.0 ;
						} else if ( this_car_lane - lane == 1 ) {
							// this vehicle is in right lane
							//set status available when no car behind 15 m and 30 m front
							right_lane_car |= car_s - 15 < check_car_s && car_s + 30 > check_car_s;
							//right_lane_car |= car_s - 15 < check_car_s && car_s + 30 > check_car_s && abs(car_d-d) > car_width/2.0;
						}
					}


					//Behavioral planning phase
					double speed_diff = 0;
					const double ref_acceleration = 0.224; //about 5m/s^2

					//if car in same lane and too close,time to change lanes
					if ( too_close ) {

						// left lane available and lane change safe
						if ( !left_lane_car && lane > 0 ) {

							lane--; // go to left of current lane
						} else if ( !right_lane_car && lane != 2 ){
							// right lane available and lane change safe
							lane++; // go to right of current lane
						} else {

							//lane change not possible, Keep Lane and reduce speed in decrements of 0.224 --> 5m/s^2
							speed_diff -= ref_acceleration;
						}
					} else {

						//logic to change to center lane if possible
						/*if ( lane != 1 ) {
							if ( ( lane == 0 && !right_lane_car ) || ( lane == 2 && !left_lane_car ) ) {
								lane = 1; // Back to center.
							}
						} */
						if ( ref_vel < target_vel ) {
							speed_diff += ref_acceleration;
						}

					}


						/*
          		//check  d of the 'other' car is within lane boundaries
          		if (d < (lane_width/2 + lane_width*lane + lane_width/2) && d > (lane_width/2 + lane_width*lane - lane_width/2)) {

          			double vx=sensor_fusion[i][3];
          			double vy=sensor_fusion[i][4];

          			//check_speed is the to control speed
          			double check_speed=sqrt(vx*vx+vy*vy);

          			//s value of the car in our lane
          			double check_car_s=sensor_fusion[i][5];

          			// if using prev points can project s value out
          			//if we are using previous points we need to project as we are not there at
          			//looking at where car willl be in future

          			check_car_s += (double)prev_size*0.02*check_speed;
          			//check s values greater than our car and  gap in s values --> then take action
          			if (check_car_s > car_s && check_car_s - car_s < 30) {
          				//do some logic here, ,lower ref vel so we dont crash into the car in front
          				//could also flag to try to change lanes
          				//ref_vel=29.5; //mph
          				//set  flag and increase/decrease vel by step rather than constraining to a fixed value
          				too_close=true;


          				//if car too close time to change lane
          				//if we are in middle of right lane get to lane 0
          				//:)) strategy --> slam to left lane no matter what
          				// due to use of spline interp, there is no jerk even though lane is changed suddenly
          				if (lane > 0) {
          					lane=0;
          				}
          			}
          		} */





					//based on too close flag, decrease speed
					/* move this section to path planner point generator
          	 if (too_close) {
          		//decrement speed in steps
          		ref_vel -= 0.224; //about 5 m/s^2
          	} else if(ref_vel < target_vel)  {
          		//increment speed in steps
          		ref_vel += 0.224;
          	}*/

					//create a list of widely spaced (x,y) waypoints evenly spaced at 30m
					//will be used for interpolation of with a spline and fill it in with
					//more points

					vector<double> ptsx;
					vector<double> ptsy;


					//keep track of car, can be either car's current loc or from prev_path's end point
					//ref x,y yaw states
					//use points from prev paths for smooth transitions
					double ref_x=car_x;
					double ref_y=car_y;
					double ref_yaw=deg2rad(car_yaw);


					//if prev size is almost empty, use the car's current s as starting reference
					//when starting out there will be no previous path, use car's current localization info and assume
					//const yaw, generate another point
					if (prev_size < 2) {
						//use 2 points that make the path tangent to car
						double prev_car_x=car_x-cos(car_yaw);
						double prev_car_y=car_y-sin(car_yaw);
						ptsx.push_back(prev_car_x);
						ptsx.push_back(car_x);

						ptsy.push_back(prev_car_y);
						ptsy.push_back(car_y);
					}
					//else use the prev path's end point as starting ref
					//find heading of the car using those couple points as well
					else {
						//redefine ref state as prev path end point
						ref_x=previous_path_x[prev_size-1];
						ref_y=previous_path_y[prev_size-1];

						//and get another point from prev path
						double ref_x_prev=previous_path_x[prev_size-2];
						double ref_y_prev=previous_path_y[prev_size-2];

						ref_yaw=atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

						ptsx.push_back(ref_x_prev);
						ptsx.push_back(ref_x);

						ptsy.push_back(ref_y_prev);
						ptsy.push_back(ref_y);
					}


					//using the above starting reference
					//generate 3 points 30m apart for spline
					//In Frenet add evenly spaced points ahead of the starting ref
					//so instead of being dist_inc spaced 50 points, we are creating
					//3 points 30 metres apart, 30 is an experimental value, works fine without excessive jerk,
					//if it is reduced the jerk may be high


					vector<double> next_wp0=getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
					vector<double> next_wp1=getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
					vector<double> next_wp2=getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

					ptsx.push_back(next_wp0[0]);
					ptsx.push_back(next_wp1[0]);
					ptsx.push_back(next_wp2[0]);

					ptsy.push_back(next_wp0[1]);
					ptsy.push_back(next_wp1[1]);
					ptsy.push_back(next_wp2[1]);

					//ptsy.ptsx has 2 pts from prev path and 3 new pts 30 metres apart
					//so 5 anchor points

					//transform to this car's local coordinates
					//we shift it so we make sure that the car is at last point of the prev
					//path is at 0,0, the origin and its angles at zero degrees
					//use homogenous transform
					for (int i=0;i<ptsx.size();i++) {
						//shift car's reference angle to 0 deg

						//first shift
						double shift_x=ptsx[i]-ref_x;
						double shift_y=ptsy[i]-ref_y;

						//then rotate
						ptsx[i]=shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw);
						ptsy[i]=shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw);

					}



					//create a spline
					tk::spline s;

					//set (x,y) points to the spline
					s.set_points(ptsx,ptsy);



					//Define the actual x,y points we will use for the planner

					json msgJson;

					vector<double> next_x_vals;
					vector<double> next_y_vals;


					//if there are points left in previous path, add them to path planner
					//Start with all of the previous path points from last time
					for (int i=0;i<previous_path_x.size();i++){
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}


					//calculate how to break up spline points so that we travel
					//at desired ref vel



					//to calculate spacing between waypoints to control speed
					//to figure out number of way points,suppose we want N points between 30 metre point
					//we know that simulator upadtes every 20 ms and velocity is ref_vel
					//distance_horizon=N*time*vel --> N=dist/time*vel
					//30 metre in x dir, break it down into N and find Y val using spline


					//horizon=30
					double target_x=30;
					//our target distance, for horizon x axes val
					double target_y=s(target_x);

					//use trig to find hypotenuse that's distance_horizon
					double target_dist=sqrt(target_x*target_x + target_y*target_y);


					//we start at the origin in cars local coord as we have done homogenous transform
					double x_add_on=0;

					//fill up the rest of our path planner after filling it with previous points
					//here the max length of path planner trajectory is 50(path_length)


					//NOTE: previous_path_x reports only the remaining path points in previous path supplied by planner
					//at each step the size reduces



					for(int i=0;i<=path_length-previous_path_x.size();i++){

						/*if (too_close) {
          			//decrement speed in steps
          			ref_vel -= 0.224; //about 5 m/s^2
          		} else if(ref_vel < target_vel)  {
          			//increment speed in steps
          			ref_vel += 0.224;
          		} */

						ref_vel += speed_diff;

						//don't go above target_vel and below ref_acceleration
						if ( ref_vel > target_vel ) {
							ref_vel = target_vel;
						} else if ( ref_vel < ref_acceleration ) {
							ref_vel = ref_acceleration;
						}
						double N=target_dist/(0.02*ref_vel/2.24); //ref_vel is in miles ph, to convert to meter ps, so factor of 1/2.24
						//--> 0.44 m per second

						//x_point is the next distance from x_add_on which was origin at start
						double x_point=x_add_on+target_x/N;
						double y_point=s(x_point);

						x_add_on=x_point;

						double x_ref=x_point;
						double y_ref=y_point;

						//rotate back to normal after rotating it earlier
						//rotate
						x_point=x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
						y_point=x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

						//then shift
						x_point +=ref_x;
						y_point +=ref_y;

						//add to next_vals

						next_x_vals.push_back(x_point);
						next_y_vals.push_back(y_point);
					}



					/*
					 * test starter code from Q&A session
          	//define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	//test simulator visuals
          	//close to 50 mph speed (0.2 ms sync time with simulator)
          	double dist_inc = 0.5;

          	for(int i = 0; i < 50; i++)
          	{
          		//50 straight line points in heading direction
          		//next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
          		//next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));

          		//50 points with middle lane,d=1.5*lane_width and s=car_s+i+1*dist_inc
          		//convert to xy coordinates

          		double next_s=car_s + (i+1)*dist_inc;
          		double next_d = 1.5*lane_width;
          		vector<double> xy=getXY(next_s,next_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          		next_x_vals.push_back(xy[0]);
          		next_y_vals.push_back(xy[1]);


          	} */



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
