#include "planning.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>

using std::string;
using std::vector;

// found the adjacent cars on certain lane
vector<vector<double>> check_cars(int lane,int &previous_size, const vector<vector<double>> &sensor_fusion){
  vector<vector<double>> adjacent_cars;
  for(int i = 0; i<sensor_fusion.size();i++){
    float d = sensor_fusion[i][6];
    if((d<(2+4*next_lane+2)) && (d>(2+4*next_lane-2))){
      double check_car_s = sensor_fusion[i][5];
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      check_car_s += ((double)previous_size*0.02*check_speed);
      
      if(fabs(check_car_s - car_s)<50){
        adjacent_cars.push_back(sensor_fusion[i]);
      } 
    }
  }
  return adjacent_cars;
}

// calculate the inefficiency cost according to the lane speed
float inefficiency_cost(int next_lane, double &ref_v,int &previous_size, const vector<vector<double>> &sensor_fusion){
  int i_cost = 0;
  vector<vector<double>> adjacent_cars = check_cars(next_lane, sensor_fusion);
  
  //if the lane is empty, cost = 0
  if(adjacent_cars.size() ==0){
    return i_cost;
  }
  //else check if there's any car in the front and if closest car in the front is faster
  else{
    double min_dis = 100000000;
    vector<double> target_car = {};
    double target_speed;
    for(int j=0; j<adjacent_cars.size(); j++){
      double check_car_s = adjacent_cars[j];[5];
      double vx = adjacent_cars[j][3];
      double vy = adjacent_cars[j][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      check_car_s += ((double)previous_size*0.02*check_speed);
      if((check_car_s > car_s) && (check_car_s - car_s)<min_dis){
        min_dis = check_car_s - car_s;
        target_car = close_cars[j];
        target_speed = check_speed;
      }
    }
    if(target_car.size() ==0){
      return i_cost;
    }
    else{
      //if target lane is faster, add cost according to the speed
      if(target_speed>ref_v){
        return i_cost += ref_v/target_speed;
      }
      //if the target lane is slower, add large cost
      else{
        return i_cost +=300;
      }
    }
  }
}

//calculate the safety cost 
float safety_cost(int target_lane, int &previous_size, const vector<vector<double>> &sensor_fusion){
  int s_cost = 0;
  //if the target lane is out of the 3 right lane, then add very big cost to avoid accident
  if(target_lane<0 || target_lane>2){
    return s_cost += 10000;
  }
  // found adjacent cars
  vector<vector<double>> adjacent_cars = check_cars(target_lane, sensor_fusion);
  
  //check if there's any car is too close to ego car
  for(int j=0; j<adjacent_cars.size(); j++){
    double check_car_s = adjacent_cars[j];[5];
    double vx = adjacent_cars[j][3];
    double vy = adjacent_cars[j][4];
    double check_speed = sqrt(vx*vx+vy*vy);
    check_car_s += ((double)previous_size*0.02*check_speed);
    if(fabs(check_car_s-car_s)<25){
      return s_cost += 10000;
    }
  }
  return s_cost;
}

float choose_next_lane(int &lane,double ref_v, int &previous_size, const vector<vector<double>> &sensor_fusion){
  int next_lane;
  double min_cost=100000000000.0;
  for(int i=-1; i<2; i++){
    int target_lane = lane + i;
    cost = inefficiency_cost(target_lane, ref_v,previous_size, sensor_fusion) + safety_cost(target_lane, previous_size, sensor_fusion);
    if(cost<min_cost){
      min_cost = cost;
      next_lane = target_lane;
    }
  }
  return next_lane;
  
}