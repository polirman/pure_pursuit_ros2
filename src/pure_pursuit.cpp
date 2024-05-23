#include "pure_puresuit.hpp"
void PurePursuit::FillPoints(std::vector<double> &local_path_x, std::vector<double> &local_path_y, const double &length) {
    if (local_path_x.size() < 2)
        return;
    double interval = sqrt(pow(local_path_x[local_path_x.size() - 1] - local_path_x[local_path_x.size() - 2], 2)
                           + pow(local_path_y[local_path_y.size() - 1] - local_path_y[local_path_y.size() - 2], 2));
    interval = std::max(interval, 0.5);
    int max_points = int(length / interval + 1);
    for (int i = 0; i < max_points; ++i) {
        local_path_x.emplace_back(2 * local_path_x[local_path_x.size() - 1] - local_path_x[local_path_x.size() - 2]);
        local_path_y.emplace_back(2 * local_path_y[local_path_y.size() - 1] - local_path_y[local_path_y.size() - 2]);
    }
}
bool PurePursuit::solve(std::vector<Eigen::Vector2d> &points,  const double &velocity, double &steer_next) {
    double solve_limit_v{};
    local_path_x.clear();
    local_path_y.clear();
    local_path_x.clear();
    local_path_y.clear();
    for (int i = 0; i < (points.size() - 1); i++) {
        local_path_x.emplace_back(points[i].x());
        local_path_y.emplace_back(points[i].y());
    }
    if (local_path_x.size() < 2 || local_path_x.size() != local_path_y.size()) {
        std::cout << " local path size error " << std::endl;
        return false;
    }
    FindNearPoint();
    PrintError();
    CalK(local_path_x, local_path_y);
    CalLimitV();
    //params.k=0.5;
    this->lookahead_distance = std::max(params.k1 * velocity + params.k2 * fabs(average_k[near_point]), 1.);
    /*std::cout<<"lookahead_distance="<<this->lookahead_distance<<std::endl;*/
    std::pair<double, double> goal;
    if (CalculateDistance(local_path_x.front(), local_path_y.front()) > lookahead_distance)
        goal = {local_path_x[1], local_path_y[1]};
    else {
        if (CalculateDistance(local_path_x.back(), local_path_y.back()) < lookahead_distance)
            PurePursuit::FillPoints(this->local_path_x, this->local_path_y, 20);
        int near_index = FindWayPoint();
        goal = FindGoalPoint(near_index);
    }
    std::cout << " goal.x : " << goal.first << " , goal.second : " << goal.second << std::endl;
    double distance = CalculateDistance(goal.first, goal.second);
    if ( solve_limit_v < 1) { solve_limit_v = 1;}
    else { solve_limit_v = velocity; }
    steer_compensate = params.k3 *  (2 * cross_error) / (distance * distance * solve_limit_v);
    steer_curv = (2 * goal.second) / (distance * distance) + steer_compensate;
    std::cout << " steer_compensate : " << steer_compensate << std::endl;
    std::cout << " std_curv : " << steer_curv << std::endl;
    if (steer_curv > params.max_steer) {
        steer_curv = params.max_steer;
    }
    if (steer_curv < -params.max_steer) {
        steer_curv = -params.max_steer;
    }
    steer_next = steer_curv;
    std::cout << " steer_next : " << steer_next << std::endl;
    return true;
}
int PurePursuit::FindWayPoint() {
    int goal_point_index = int(local_path_x.size() - 1);
    for (int i = 0; i + 1 < local_path_x.size(); ++i) {
        double min_distance = CalculateDistance(local_path_x.at(i), local_path_y.at(i));
        double max_distance = CalculateDistance(local_path_x.at(i + 1), local_path_y.at(i + 1));
        if (min_distance <= lookahead_distance && max_distance > lookahead_distance)
            goal_point_index = i;
    }
    return goal_point_index;
}
std::pair<double, double> PurePursuit::FindGoalPoint(int index) {
    if (index == local_path_x.size() - 1)
        return std::make_pair(local_path_x.back(), local_path_y.back());
    // (local_path_x[index], local_path_y[index]), (local_path_x[index+1], local_path_y[index+1])
    std::pair<double, double> point1 = {local_path_x[index], local_path_y[index]};
    std::pair<double, double> point2 = {local_path_x[index + 1], local_path_y[index + 1]};
    double dis = CalculateDistance((point1.first + point2.first) / 2., (point1.second + point2.second) / 2.);
    while (fabs(dis - lookahead_distance) >= 0.01) {
        if (dis > lookahead_distance)
            point2 = {(point1.first + point2.first) / 2., (point1.second + point2.second) / 2.};
        else if (dis < lookahead_distance)
            point1 = {(point1.first + point2.first) / 2., (point1.second + point2.second) / 2.};
        dis = CalculateDistance((point1.first + point2.first) / 2., (point1.second + point2.second) / 2.);
    }
    return std::make_pair((point1.first + point2.first) / 2., (point1.second + point2.second) / 2.);
}
void PurePursuit::PrintError() {
    angle = 0;
    dex = 0;
    for (int i = 0; i < (local_path_x.size() - 1); i++) {
        double min;
        min = fabs(local_path_x[0]);
        if (fabs(local_path_x[i]) < min && (local_path_x[i + 1] - local_path_x[i]) > 0) {
            dex = i;
            std::cout << " dex " << dex << std::endl;
        }
    }
    angle = (0 - atan(((local_path_y[dex + 1] - local_path_y[dex]) / fabs(local_path_x[dex + 1] - local_path_x[dex]))));

//    cross_error = local_path_y[dex];
    cross_error = distance_near_point;
    angle_error = angle;
//    std::cout << " 横向偏差 ：  " << local_path_y[dex] << std::endl;
    std::cout << " 横向偏差 ：  " << distance_near_point << std::endl;
    std::cout << " 航向角偏差 ：  " << angle << std::endl;
}
void PurePursuit::CalK(std::vector<double> &local_path_x, std::vector<double> &local_path_y) {
    s = 0;
    limit_v = 0;
    n = 0;
    ds.clear();
    k.clear();
    average_k.clear();
    std::vector<double> accurate_delta_angle{};
    std::vector<int> sequence_delta_angle{};
    for (int i = 0; i < local_path_x.size() - 1 - dex; ++i) {
        n = i;
        ds.emplace_back(CalculateDistance((local_path_x[dex + i + 1] - local_path_x[dex + i]),
                                          (local_path_y[dex + i + 1] - local_path_y[dex + i])));
        s = s + ds[i];
        if (i < (local_path_x.size() - 2 - dex)) {
            double angle1, angle2;
            auto delta_y1 = local_path_y[dex + i + 1] - local_path_y[dex + i];
            auto delta_x1 = local_path_x[dex + i + 1] - local_path_x[dex + i];
            auto delta_y2 = local_path_y[dex + i + 2] - local_path_y[dex + i + 1];
            auto delta_x2 = local_path_x[dex + i + 2] - local_path_x[dex + i + 1];
            angle1 = std::atan2(delta_y1, delta_x1);
            angle2 = std::atan2(delta_y2, delta_x2);
            double delta_angle = angle2 - angle1;

            constexpr auto D2_PI = M_PI * 2.;
            while (delta_angle > M_PI) {
                delta_angle -= D2_PI;
            }
            while (delta_angle < -M_PI) {
                delta_angle += D2_PI;
            }
            accurate_delta_angle.emplace_back(delta_angle);
            k.emplace_back(delta_angle / ds[i]);
//            std::cout << i << " delta_angle: " << delta_angle << " / " << delta_angle * 180. / M_PI << " ds: " << ds[i] << " " << k.back() << std::endl;


//            if (delta_x1 < 0 && delta_x2 < 0) {
//                if (delta_y1 * delta_y2 < 0) {
//                    if (delta_y1 < 0) {
//                        k.emplace_back(-(2 * 3.14 - fabs(angle1) - fabs(angle2)) / ds[i]);
//                    } else {
//                        k.emplace_back((2 * 3.14 - fabs(angle1) - fabs(angle2)) / ds[i]);
//                    }
//                } else {
//                    k.emplace_back((angle1 - angle2) / ds[i]);
//                }
//            } else {
//                k.emplace_back((angle1 - angle2) / ds[i]);
//                if (k[i] > 5) {
//                    std::cout << " angle1 : " << angle1;
//                    std::cout << " ; angle2 : " << angle2 << std::endl;
//                    std::cout << " delta_y1 : " << delta_y1;
//                    std::cout << " ; delta_y2 : " << delta_y2 << std::endl;
//                    std::cout << " delta_x1 : " << delta_x1;
//                    std::cout << " ; delta_x1 : " << delta_x2 << std::endl;
//                }
//            }
        } else {
            k.emplace_back(0);
        }
        if (s > limit_s) {
            i = local_path_x.size() - 1 - dex;
        }
    }
    sequence_delta_angle = NoiseReduce( accurate_delta_angle );
    std::sort( sequence_delta_angle.begin(),sequence_delta_angle.end(),std::greater<size_t>() );
/*    for (int i = 0; i < sequence_delta_angle.size(); i++)
    {
        std::cout << "sequence_delta_angle " << sequence_delta_angle[i] << std::endl;
    }*/
    for ( size_t pos : sequence_delta_angle) {
        k.erase(k.begin() + pos);
    }
//67 63 57 52 10 6 0
//    std::cout << " sequence_delta_angle,size :  " << sequence_delta_angle.size() << std::endl;
    if (n > 2) {
        for (int i = 0; i < n - 2; i++) {
            double K1, K2, K3, Ka;
            Ka = ds[i] + ds[i + 1] + ds[i + 2];
            K1 = ds[i] / Ka;
            K2 = ds[i + 1] / Ka;
            K3 = ds[i + 2] / Ka;
            average_k.emplace_back(K1 * k[i] + K2 * k[i + 1] + K3 * k[i + 2]);
        }
    }
}
void PurePursuit::CalLimitV() {
    if (average_k.size() > 2) {
        int j = 0;
        max_k = 0;
        for (int i = 0; i < average_k.size(); i++) {
            double MostK ;
            if (i < average_k.size() - 2 && i > 1) {
                MostK = fabs(average_k[i - 2]) + fabs(average_k[i - 1]) + fabs(average_k[i + 1]) + fabs(average_k[i + 2]);
            } else { MostK = fabs(average_k[i]); }
            if (fabs(average_k[i]) > fabs(average_k[j])) {
                j = i;
            }
/*            if (fabs(average_k[i]) > 5 * MostK) {
                std::cout << "average_k[i] :" << average_k[i] << std::endl;
                std::cout << "5 * MostK : " << 5 * MostK << std::endl;
                std::cout << " average_k[" << i - 2 << "] " << average_k[i - 2] << std::endl;
                std::cout << " average_k[" << i - 1 << "] " << average_k[i - 1] << std::endl;
                std::cout << " average_k[" << i << "] " << average_k[i] << std::endl;
                std::cout << " average_k[" << i + 1 << "] " << average_k[i + 1] << std::endl;
                std::cout << " average_k[" << i + 2 << "] " << average_k[i + 2] << std::endl;
            }*/
        }
        //std::cout << " j :" << j << std::endl;
        //std::cout << j << " AVERK : " << average_k[j] << std::endl;
        if (fabs(average_k[j] )> params.max_steer) { max_k = average_k[j] / fabs(average_k[j] ) * params.max_steer; }
        else { max_k = average_k[j]; }
        limit_v = sqrt(a / fabs(max_k));
    } else {
        limit_v = 0;
    }
}
//return noise position
std::vector<int> PurePursuit::NoiseReduce(std::vector<double> x) {
    std::vector<int> y{}; double difference_f4,difference_i,average_f4;
    if ( x.size() > 2){
        for ( int i = 2; i < x.size()-2; i++) {
            average_f4 = ( x[i-2] + x[i-1] + x[i+1] + x[i+2] ) / 4;
            difference_f4 = fabs( x[i-2] - average_f4) + fabs( x[i-1] - average_f4) + fabs( x[i+1] - average_f4) + fabs( x[i+2] - average_f4);
            difference_i = fabs(x[i] - average_f4);
            if ( difference_i > difference_f4 ) {
                y.emplace_back(i);
            }
        }
//        std::cout << " y  : " << std::endl;
/*        for ( size_t i : y ) {
            std::cout << i << std::endl;
        }*/
        return y;
    } else {
//        std::cout << " k.size < 2" << std::endl;
        return y;
    }
}

void PurePursuit::FindNearPoint() {
    near_point = 0;
    distance_near_point = CalculateDistance( local_path_x[ near_point ], local_path_y[ near_point ] );
    double tmp_distance = 0;
    for ( size_t i = 1; i <  local_path_x.size(); i++) {
        tmp_distance = CalculateDistance( local_path_x[i],local_path_y[i]);
        if ( fabs(tmp_distance) < fabs(distance_near_point) ) {
            near_point = i ;
        }
    }
    std::cout << " near_point : "<< near_point << std::endl;
    distance_near_point = distanceToOrigin( local_path_x[ near_point ],local_path_y[ near_point ], local_path_x[ near_point + 1 ], local_path_y[ near_point + 1 ] );
}
double PurePursuit::distanceToOrigin(double x1, double y1, double x2, double y2) {
    double a = y2 - y1;
    double b = x1 - x2;
    double c = x2 * y1 - x1 * y2;
    int d = 1;
    if (y2 < 0){ d = -1;}
    else { d = 1; }
    double distance = d * fabs(c) / sqrt(a * a + b * b);
    return distance;
}