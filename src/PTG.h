#ifndef PTG_H
#define PTG_H

#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"


using namespace std;

class PTG {
public:
	PTG();

	virtual ~PTG();

  int Lane(int lane, double car_s, double car_d, double car_speed, vector<vector<double>> &cars_in_range, vector<vector<double>> &cars_in_front, double prev_size);

private:
  double slower_car_in_lane_cost(int lane, vector<vector<double>> &cars_in_lane_front_i, double car_s, double prev_size);
  double lane_switching_cost(int current_lane, int new_lane);
  double collision_cost(int car_s, vector<vector<double>> &cars_in_range, double prev_size);
};


#endif /* PTG_H */