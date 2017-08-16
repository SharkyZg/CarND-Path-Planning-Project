#include "PTG.h"

PTG::PTG() {}
PTG::~PTG() {}

// Return optimal lane based on cars cars in range
int PTG::Lane(int lane, double car_s, double car_d, double car_speed, vector<vector<double>> &cars_in_range, vector<vector<double>> &cars_in_front, double prev_size) {
  double lane_cost[3]; // Array with 
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
  
  memset(lane_cost, 0, sizeof(lane_cost)); 
  
  return optimal_lane;
  }

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

// Function to add cost in case of switching lanes. 
double PTG::lane_switching_cost(int current_lane, int new_lane)
{
  if(current_lane == new_lane) {return 0;} else return 5;
}

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