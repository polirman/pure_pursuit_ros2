//
// Created by polir on 24-2-19.
//

#ifndef MOTION_CONTROL_PURE_PURE_PURESUIT_HPP
#define MOTION_CONTROL_PURE_PURE_PURESUIT_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>  // Corrected message type
#include <GeographicLib/LocalCartesian.hpp>
#include <chrono>
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "autonomous_proto.hpp"
#include "autonomous_proto.hpp"

struct PPParams {
    double k1{},k2{},k3{};
    double max_steer{};
};

class PurePursuit {
public:
    double angle{};
    double cross_error{},angle_error{};
    void CalK(std::vector<double> &local_path_x, std::vector<double> &local_path_y);
    PurePursuit() : steer_curv(0.) {}
    std::vector<double> k{},average_k{};
    std::vector<double> ds{};
    double s = 0, limit_s = 0, n = 0,max_k=0;
    bool solve(std::vector<Eigen::Vector2d> &points,
               const double &velocity,
               double &steer_next);
    void FindNearPoint();
    int near_point{};
    double distance_near_point = 0;
    double distanceToOrigin(double x1, double y1, double x2, double y2);
    PPParams params{};
    double limit_v{},a{};
    static void FillPoints(std::vector<double> &local_path_x, std::vector<double> &local_path_y, const double &length) ;
private:
    static inline double CalculateDistance(const double &x1, const double &y1) { return sqrt(pow(x1, 2) + pow(y1, 2)); }
    int FindWayPoint();
    std::pair<double, double> FindGoalPoint(int index);
    void PrintError();
    void CalLimitV();
    std::vector<int> NoiseReduce(std::vector<double> x);
    int dex{};


    std::vector<double> local_path_x{};
    std::vector<double> local_path_y{};
    std::vector<double> transfer_accurate_k{};
    double steer_curv{},steer_compensate{};
    double lookahead_distance{};
};






#endif //MOTION_CONTROL_PURE_PURE_PURESUIT_HPP
