
# Path Planning Project
## Problem Description

In this project we are focusing on self driving car trajectory generation in the Udacity's self driving car simulator. Self driving car is driving on three lane highway in the simulator. The goal is to drive the car as smooth as possible while maintaining the maximal speed of 50 miles per hour. To do that without any accidents it is necessary to pass other cars and adopt speed when necessary. Other important concern is to change lanes or speed as smooth as possible, so that passengers do not feel unsafe or uncomfortable.

## Model Description

C++ model solves the described problem of path generation. The model receives input data from the Udacity's simulator with car coordinates, speed, direction and other cars. Based on this input and map information the model generates a path of 50 coordinates that it sends back to the simulator. Those coordinates represent path that the car will follow until next loop with new set of coordinates.

## Map Information

The CSV file highway_map.csv contains all the map information necessary. This information contains cartesian and freenet coordinates of the road. We can calculate coordinates of a specific lane by calcuating freenet d, for example 2+4*1 is freenet d for the middle lane.
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  
  // Parse input data.
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
## Data Parsing

The simulator sends JSON object containing car coordinates in freenet and cartesian system, car direction, speed, previous path that is not processed yet and information from other cars on the road (sensor fusion). Those coordinates are then being parsed in appropriate varaibles and arrays.
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
vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
It is necessary to loop through all cars in sensor_fusion array to determine which cars are in our lane and take appropriate actions, slow down or switch lane if necessary.

We will add some cars to cars_in_front and cars_in_range arrays as well, this arrays will later be used to determine optimal lane.
for(int i = 0; i < sensor_fusion.size(); i++)
{
  double vx = sensor_fusion[i][3];
  double vy = sensor_fusion[i][4];
  double check_speed = sqrt(vx*vx+vy*vy);
  double check_car_s = sensor_fusion[i][5];
  float d = sensor_fusion[i][6];       
  check_car_s += ((double)prev_size*.02*check_speed);

  if(check_car_s > car_s ){
    cars_in_front.push_back(sensor_fusion[i]);
    //car is in my lane

    if(d < (2+4*lane+2) && d > (2+4*lane-2)){
        //check s values grater than mine and s gap
        if((check_car_s-car_s) < 30) // If front car is in 30 m break and change lane.
        {
          too_close = true;
          switch_lane = true;
        }
        if((check_car_s-car_s) < 5) // If front car is in 5 m break double.
        {
          way_too_close = true;
        }
        else if(check_car_s-car_s < 80 ) // If front car is in 80 m change lane if possible.
        { 
          switch_lane = true;
        }   
      }

  }

  // Add cars in range for later colision avoidance detection.
  vector<double> other_car_xy = getXY(check_car_s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  if(distance(other_car_xy[0], other_car_xy[1], car_x, car_y) < 100) {
    cars_in_range.push_back(sensor_fusion[i]);
  }

}
## Speed Control

Based on the too_close and way_too_close booleans set during sensor_control array loop we control the desired speed (ref_vel variable).

Reference velocity - ref_vel is increased or decreased incrementally to avoid high jerk during acceleration or breaking.
// Adopt velocity according to previously defined booleans.
if(too_close)
{
  ref_vel -= .224;
}
else if(ref_vel < 49.5)
{
  ref_vel += .224;
}
if(way_too_close)
{
  ref_vel -= .5;
}
## Search of Optimal Lane

If there is a slower vehicle in front of us it is desireable to switch to faster lane. This is why we set switch_lane boolean in case of slower car in front of us during sensor_fusion array loop.

The model checks every 30 simulator iterations if the switch_lane boolean is active and searches for optimal lane in case that it is.
// Find optimal lane every 30 iterations.  
if(switch_lane && iteration == 30) {
  lane = ptg.Lane(lane, car_s, car_d, car_speed, cars_in_range, cars_in_front, (double)prev_size);
}
iteration++;
if(iteration > 30) {iteration = 0;}
### PTG.Lane cost function

PTG.Lane cost function returns optimal lane and it is part of PTG class (PTG.cpp, PTG.h).

The function first separates previously collected cars_in_front and cars_in_range vectors. Each of the vecotrs is separated into three separate vectors depending on the lane in which other cars are in. As result we get 2 arrays containing three vectors each: 
cars_in_lane_front[3] - Array used to determine fastest lane.
cars_in_lane_range[3] - Array used to determine collision risk.
  vector<vector<double>> cars_in_lane_front[3]; // Create array with three different vectors with cars in front depending on lane. 
  vector<vector<double>> cars_in_lane_range[3]; //Create array with three different vectors with cars in range depending on lane. 
  int optimal_lane = lane; // Current lane
  double lowest_lane_cost = 9999;
  
  // Add cars_in_front cars in array depending on lane.
  for(int i = 0; i < cars_in_front.size();i++)
  { 
    float d = cars_in_front[i][6];
    for(int j = 0; j < 3; j++){
        if(d < (2+4*j+2) && d > (2+4*j-2))
        {
          cars_in_lane_front[j].push_back(cars_in_front[i]);
        }
      }
  }
  
  // Add cars_in_range cars in array depending on lane.
  for(int i = 0; i < cars_in_range.size();i++)
  { 
    float d = cars_in_range[i][6];
    for(int j = 0; j < 3; j++){
        if(d < (2+4*j+2) && d > (2+4*j-2))
        {
          cars_in_lane_range[j].push_back(cars_in_range[i]);
        }
      }
  }
After previously described arrays have been created, we can start calculating cost of each lane to determine lane with lowest cost. 

This is done by looping through each lane and adding costs calculated by the three cost functions described later.

It is important to note that if we need to switch 2 lanes at once, for example from the left lane to the right lane, then we need to consider collision risk in the middle lane as well.
  for(int i = 0; i < 3; i++)
  {  
    // Loop through all cars in front for each lane and add lane costs appropriately.
    if(cars_in_lane_front[i].size()>0){
      lane_cost[i] += slower_car_in_lane_cost(i,cars_in_lane_front[i],  car_s, prev_size);
    } else lane_cost[i]=0;
    
    lane_cost[i] += lane_switching_cost(lane, i); // Add small cost for switching lanes.
    
    // Loop through all cars in range for each lane and add lane costs appropriately.
    if(i!=lane){      
      if(lane==0 && i == 2) { // If new lane is two lanes apart middle lane collision risk costs should be considered.
        lane_cost[i]+=collision_cost(car_s, cars_in_lane_range[1], prev_size);
      }
      if(lane==2 && i == 0) { // If new lane is two lanes apart middle lane collision risk costs should be considered.
        lane_cost[i]+=collision_cost(car_s, cars_in_lane_range[1], prev_size);
      } 
      if(lane != i){
        lane_cost[i]+=collision_cost(car_s, cars_in_lane_range[i], prev_size);
      }
    }

  }
### slower_car_in_lane_cost Function

slower_car_in_lane_cost function runs for each lane to add higher cost to lanes with closer vehicles. Function predicts where vehicle from sensor_fusion array will be 0.2 seconds in future as well.

If a vehicle is closer to the car more cost will be added. If vehicle does not exist, then the cost will be 0.
// Function that adds costs to the lane based on speed of cars in lane.
double PTG::slower_car_in_lane_cost(int lane, vector<vector<double>> &cars_in_lane_front_i, double car_s, double prev_size) {
  double cost = 0;
  double cost_temp = 0;
  
  // Loop through cars in lane and add costs for lanes appropriately.
  for(int i = 0; i < cars_in_lane_front_i.size();i++) {
      double check_car_s = cars_in_lane_front_i[i][5]; 
      double vx = cars_in_lane_front_i[i][3];
      double vy = cars_in_lane_front_i[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      check_car_s += ((double)prev_size*.02*check_speed);
      cost_temp = 500/(check_car_s-car_s);
      
      // Keep costs only for closest car in lane.
      if(cost_temp > cost) { cost = cost_temp;}
  }
  return cost;
}
### collision_cost Function

collision_cost function ensures that we do not collide with some other vehicle during lane switch. If there is some other vehicle in lane where we plan to switch and there is collision risk with that vehicle (car_s is in range of other car's position), very high cost will be added.
// Function that adds costs to the lane based on collision risk.
double PTG::collision_cost(int car_s, vector<vector<double>> &cars_in_lane_range_i, double prev_size)
{
  double cost = 0;
  double cost_temp = 0;
  // Loop through cars in lanes and add costs appropriately.
  for(int i = 0; i < cars_in_lane_range_i.size();i++) {
      double check_car_s = cars_in_lane_range_i[i][5]; 
      double vx = cars_in_lane_range_i[i][3];
      double vy = cars_in_lane_range_i[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      check_car_s += ((double)prev_size*.02*check_speed);
    
      // Add very high cost if collision risk exist.
      if( ((check_car_s-car_s) > -15) && ((check_car_s-car_s)<15)) { cost = 10000;}
  }
  return cost;
  
}
### lane_switching_cost Function

lane_switching_cost function adds small cost for lane switch to avoid that car constantly switches lanes when costs of two lanes are basically the same.
// Function to add cost in case of switching lanes. 
double PTG::lane_switching_cost(int current_lane, int new_lane)
{
  if(current_lane == new_lane) {return 0;} else return 5;
}
### Returning Optimal Lane

At the end of the PTG.Lane cost function optimal lane with the lowest cost will be returned. 

If we need to switch 2 lanes, for example from left lane to right lane, then the middle lane will be returned first. We do this to avoid high jerk due to switching two lanes at same time and because it is not usual and expected from other drivers that the car switches two lanes at once.
for(int i = 0; i < 3; i++){  // Check which lane has the lowest point.
  if(lane_cost[i]<lowest_lane_cost) {
    optimal_lane = i;
    lowest_lane_cost = lane_cost[i];
  }
  std::cout<<"Lane cost: "<<i<<" - " <<lane_cost[i] <<std::endl;

}

// If distance between current an optimal lane is more than one lane change first to the middle lane.
if(lane==0 && optimal_lane == 2) {optimal_lane=1;} 
if(lane==2 && optimal_lane == 0) {optimal_lane=1;}

return optimal_lane;
## Path Generation

After optimal lane has been chosen, it is necessary to generate path on which the car will drive. Final path will be sent as an array of x and y coordiates to the simulator.


### Get Reference Cordinates

First we need to get starting coordinates that we will use as reference points. We will add them to ptsx and ptsy vectors. If there are no previous coordinates returned from the simulator available, we will use car's current coordinates as a starting point. If there are at least two coordinates in previous_path vectors available, we will use them as a starting point and to calculate car's steering angle (yaw rate).
// Create a list of widely spaced (x,y) waypoints, evenly spacet at 30 m
// Later we will interoplate these waypoints with a spline and fill it in with more points 

vector<double> ptsx;
vector<double> ptsy;

// reference x,y, yaw states
// either we will reference the starting point as where the car is or at the previous paths end point
double ref_x = car_x;
double ref_y = car_y;
double ref_yaw = deg2rad(car_yaw);

// if previous size is almost empty, use the car as starting reference
if(prev_size < 2)
{
  //Use two points that make the path tangent to the car
  double prev_car_x = car_x - cos(car_yaw);
  double prev_car_y = car_y - sin(car_yaw);

  ptsx.push_back(prev_car_x);
  ptsx.push_back(car_x);

  ptsy.push_back(prev_car_y);
  ptsy.push_back(car_y);
}
// use the previous path's end point as starting reference
else
{
  // Redefine reference state as previous path end point
  ref_x = previous_path_x[prev_size-1];
  ref_y = previous_path_y[prev_size-1];

  double ref_x_prev = previous_path_x[prev_size-2];
  double ref_y_prev = previous_path_y[prev_size-2];
  ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

  // Use two points that make the path tangent to the previous path's end point
  ptsx.push_back(ref_x_prev);
  ptsx.push_back(ref_x);

  ptsy.push_back(ref_y_prev);
  ptsy.push_back(ref_y);          

}
### Create List of Evenly Spaced Corrdinates

Now we will calculate waypoints in freenet coordinate system, by adding 30 meters to the "s" axis. "d" axis is calculated by multiplying current (optimal) lane by 4 and adding 2. For example "d" axis for the lane 0 (left) is 2+4*0 = 2. 

With help of the getXY function we get cartesian coordinates that can later be used by the simulator.  
// In Frenet add evenly 30m spaced points ahead of the starting reference

vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);
Before calculating new path trajectory with spline it is necessary to rotate cartesian coordinates that we have just calculated to car reference angle of 0 degrees. We do this to simplify the math.
for (int i = 0; i < ptsx.size(); i++)
{
  //shift car reference ange to 0 degrees
  double shift_x = ptsx[i]-ref_x;
  double shift_y = ptsy[i]-ref_y;

  ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y * sin(0-ref_yaw));
  ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y * cos(0-ref_yaw));
}
### Spline function

As we have created vectors with reference points for future trajectory, we need to make trajectory smooth to avoid high jerk. Another thing that we need to control is distance between trajectory points, which will affect car's speed. We do that with the help of spline function. We calculate a spline function by adding previously created ptsx and ptsy vectors: s.set_points(ptsx, ptsy); . 
// create a spline
tk::spline s;

// set (x,y) points to the spline
s.set_points(ptsx, ptsy);


First we will add returned unused points form previous path.
// Define the actual (x,y) points we will use for the planner
vector<double> next_x_vals;
vector<double> next_y_vals;

// Start with all of the previous path points from last time
for(int i = 0; i < previous_path_x.size(); i++)
{
  next_x_vals.push_back(previous_path_x[i]);
  next_y_vals.push_back(previous_path_y[i]);
}
Before we can start calculating points based on spline function, it is necessary to calculate how to convert previously defined points into spline trajectory. To do this we need to calculate target_y based on target_x of 30 meters and a resulting target distance.

After that we can start looping to add new points that need to be send back to the simulator (50 - size of previous path).

To calculate to how many points we will split our target distance of 30 meters, we will use this formula: "target_dist/(.02*ref_vel/2.24)" where ref_vel represents desired velocity. Then we can divide target_x with resulting number N and add this value to previous x_point(x_add_on) so that we calculate next x_point. When we insert this newly calculated x_point to spline function "s", we are able to calculate y_point value as well.
// Calculate how to break up spline points so that we travel at our desired reference velocity
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

double x_add_on = 0;

// Fill up the rest of our path planner after filling it with previous points

for (int i = 1; i<= 50 - previous_path_x.size(); i++) {

  double N = (target_dist/(.02*ref_vel/2.24));
  double x_point = x_add_on+(target_x)/N;
  double y_point = s(x_point);

  x_add_on = x_point;

  double x_ref = x_point;
  double y_ref = y_point;


The last thing that we need to address before sending data to the simulator is to rotate new trajectory points back to standard coordinates and to add them to next_x_vals and next_y_vals vectors.

  // rotate back to normal after rotating it earlier
  x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
  y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

  x_point += ref_x;
  y_point += ref_y;

  next_x_vals.push_back(x_point);
  next_y_vals.push_back(y_point);
}

### Returning Data to the Simulator

Now that we have prepared vectors with new trajectory data we need to send it to the simulator by using ws.send function.
json msgJson;

// add new path points that will be send to the simulator.
msgJson["next_x"] = next_x_vals;
msgJson["next_y"] = next_y_vals;

auto msg = "42[\"control\","+ msgJson.dump()+"]";

//this_thread::sleep_for(chrono::milliseconds(1000));
ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
## Conclusion

I found the path planning project very interesting as this was the missing link to connect all the things that we learned during the previous projects. By being able to create desired trajectories based on maps and sensor fusion inputs I feel that I am one step closer to become a self driving car engineer.

Some of the things that could still be improved in the project are better predictions where other cars will go in future and better reaction to unforeseen situations by avoiding collisions when other car unexpectedly gets in our lane for example.


```python

```
