#ifndef PLANNINNG_H
#define PLANNINNG_H


using std::map;
using std::string;
using std::vector;

vector<vector<double>> check_cars(int lane,int &previous_size, const vector<vector<double>> &sensor_fusion);

float inefficiency_cost(int next_lane, double &ref_v,int &previous_size, const vector<vector<double>> &sensor_fusion);

float safety_cost(int target_lane, int &previous_size, const vector<vector<double>> &sensor_fusion);

float choose_next_lane(int &lane,double ref_v, int &previous_size, const vector<vector<double>> &sensor_fusion);

#endif