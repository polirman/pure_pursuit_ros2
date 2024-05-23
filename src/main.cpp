#include <chrono>
#include "pure_puresuit.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
//using std::placeholders::_1;
class MotionControlPure : public rclcpp::Node  {
public:
    MotionControlPure() : Node("motion_control"), count_(0) {
    RCLCPP_INFO(get_logger(), "Publisher created for 'control' topic");
    start_time = std::chrono::high_resolution_clock::now();
    this->declare_parameter<double>("my_parameter", 0.0);
    sleep(1);
    destruct_stauts = true;
    }
    size_t count_;
    int mode{};
    double t_v{},v_set = 0;
    std::vector<Eigen::Vector2d> origin_points{};
    void CalculateCor();
    bool PushPoint(const double &in_x, const double &in_y);
    void Process();
    void FillData();
    void ReadYaml();
    void SetV();
private:
    bool destruct_stauts = false;
    double tmp_v=0,tmp_a=0,tmp_th=0,tmp_brake=0,tmp_max_steer=0;
    double v=0,e=0,steer=0,my_parameter_=0;
    bool status_test = true,stable_time = false;
    bool v_set01 = true,v_set12 = true,v_set23 = true,v_set32 = true,v_set21 = true,v_set10 = true;
    std_msgs::msg::Float64MultiArray msg_pure{};
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time{};
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_error{};
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_{};
    std::chrono::time_point<std::chrono::high_resolution_clock> end_time{};
    std::chrono::milliseconds dur_time{};
    std::chrono::milliseconds dur_time_error{};
    //start_time start_time_error start_time_  dur_time_error
    autonomous_proto::LocalPath lp{};
    autonomous_proto::Navigation nav{};
    autonomous_proto::VehicleState vs{};
    autonomous_proto::Control control_proto;
    std_msgs::msg::UInt8MultiArray local_path_msg;
    double steer_next=0;
    double wheel_base{};
    //rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    PurePursuit test;
    void LocalPath(const std_msgs::msg::UInt8MultiArray::ConstSharedPtr& msg) {
            std::cout << std::endl;
            std::cout << "-------------------------------------------" << std::endl;
            std::cout << "-------------------------------------------" << std::endl;
        if (destruct_stauts) {
            local_path_msg = *msg;
            lp.clear_direction();
            lp.clear_header();
            lp.clear_points();
            static auto publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("control", 1);
            static auto publisher_e = this->create_publisher<std_msgs::msg::Float64MultiArray>("CrossError", 1);
            if (lp.ParseFromArray(msg->data.data(), msg->data.size())) {
                std::cout << " have recieved local_path " << std::endl;
            } else { std::cout << " failed recieve local_path " << std::endl; }
            origin_points.clear();
            ReadYaml();
            CalculateCor();
            if (vs.v().size() == 0) {
                v = 0;
                std::cout << " vs.v.size() input = 0" << std::endl;
            } else {
                v = vs.v().Get(vs.v().size() - 1);
                std::cout << " 车辆速度v： " << v << std::endl;
            }
            my_parameter_ = this->get_parameter("my_parameter").as_double();
            SetV();
            test.params.max_steer = tmp_max_steer;
            control_proto.Clear();
            test.solve(origin_points, v, steer_next);
            steer = atan(steer_next * wheel_base);
            std::cout << " v : " << v << ";      LimitV : " << test.limit_v << std::endl;
            if (steer > test.params.max_steer) {
                steer = test.params.max_steer;
            }
            std::cout << " steer : " << steer << std::endl;
            if (v > test.limit_v) {
                tmp_v = test.limit_v;
                std::cout << "v-Limitv : " << v - test.limit_v << std::endl;
            }
            FillData();
            static std_msgs::msg::UInt8MultiArray msgs;
            msgs.data.clear();
            msgs.data.resize(control_proto.ByteSizeLong());
            control_proto.SerializeToArray(msgs.data.data(), msgs.data.size());
            publisher_->publish(msgs);
            msg_pure.data.clear();
            //local_path_lat and lon,nav lat and lon,error_cross and heading,v_except and actual
            msg_pure.data.push_back(lp.points()[test.near_point].lat().value());
            msg_pure.data.push_back(lp.points()[test.near_point].lon().value());
            msg_pure.data.push_back(nav.position().lat().value());
            msg_pure.data.push_back(nav.position().lon().value());
            msg_pure.data.push_back(test.cross_error);
            msg_pure.data.push_back(v_set);
            msg_pure.data.push_back(vs.v().Get(vs.v().size() - 1));
            msg_pure.data.push_back(v_set - vs.v().Get(vs.v().size() - 1));
            msg_pure.data.push_back(steer);
            msg_pure.data.push_back(vs.angle().Get(vs.angle().size()-1));
            msg_pure.data.push_back(test.angle);
            //auto message = std::make_unique<std_msgs::msg::Float64>();
            //message->data = test.cross_error;
            publisher_e->publish(msg_pure);
        }
    }
    void Navigation(const std_msgs::msg::UInt8MultiArray::ConstSharedPtr& msg){
        std::cout<<"nav parseFromArray"<<std::endl;
        nav.Clear();
        /*nav.clear_acceleration();nav.clear_header();nav.clear_angular_acceleration();
        nav.clear_angular_velocity();nav.clear_orientation();nav.clear_position();nav.clear_velocity();*/
        if(not nav.ParseFromArray(msg->data.data(), msg->data.size())){
            std::cout<<"nav parseFromArray failed"<<std::endl;
        }
    }
    void Vehicle_state(const std_msgs::msg::UInt8MultiArray::ConstSharedPtr& msg){
        std::cout<<"vs parseFromArray "<<std::endl;
        vs.Clear();
        if(not vs.ParseFromArray(msg->data.data(), msg->data.size()))
        {
            std::cout<<"vs parseFromArray failed"<<std::endl;
        }
        //  RCLCPP_INFO(get_logger(), "Vehicle_state");
    }
};
int main(int argc, char **argv) {
    std::cout << "Entering main function" << std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionControlPure>();
    node->Process();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
void MotionControlPure::CalculateCor() {
    static GeographicLib::LocalCartesian proj{};
    /*std::cout<<" nav.lat() : " << nav.position().lat().value() << std::endl;
    std::cout<<" nav.lon() : " << nav.position().lon().value() << std::endl;*/
    proj.Reset(nav.position().lat().value(), nav.position().lon().value(), nav.position().alt().value());
    static double Cos = 0, Sin = 0, e = 0, n = 0, u = 0, x = 0, y = 0;
    Cos = cos(nav.orientation().yaw().value());
    Sin = sin(nav.orientation().yaw().value());
    //    std::cout<<" nav.yaw() = "<< nav.orientation().yaw().value() <<std::endl;
    for (const auto &point: lp.points()) {
        proj.Forward(point.lat().value(), point.lon().value(), point.alt().value(), e, n, u);
        x = Cos * e + Sin * n;
        y = -Sin * e + Cos * n;
        //  std::cout<<"x="<<x<<"y="<<y<<std::endl;
        PushPoint(x, y) ;
    }
    //std::cout<<"point点输出完成"<<std::endl;
}
bool MotionControlPure::PushPoint(const double &in_x, const double &in_y){
    origin_points.emplace_back(in_x,in_y);
    return true;
}
void MotionControlPure::FillData() {
        control_proto.set_mode(autonomous_proto::Control_Mode_Value_autonomous);
        control_proto.add_direction(autonomous_proto::Control_Direction_Value_forward);
        if(lp.direction()==autonomous_proto::LocalPath_Direction_Value_forward){
            if ( v == 0 ) { steer = 0; }
            control_proto.add_k(steer);
            //control_proto.add_k(1);
            std::cout << " mode : " << mode << std::endl;
            if ( mode == 1) {
            control_proto.add_v(tmp_v);
        }
        else {
            control_proto.add_v(v_set);
            std::cout << " v_set : " << v_set << std::endl;
        }
        control_proto.add_acc(tmp_a);
        control_proto.add_throttle(tmp_th);
        control_proto.add_brake(tmp_brake);
         }
        else{
           control_proto.add_k(0);
           control_proto.add_v(tmp_v);
           control_proto.add_acc(tmp_a);
           control_proto.add_throttle(tmp_th);
           control_proto.add_brake(tmp_brake);
        }
}
void MotionControlPure::Process() {

    static auto sub_vehicle_state = this->create_subscription<std_msgs::msg::UInt8MultiArray>("vehicle_state", 1, std::bind(
            &MotionControlPure::Vehicle_state, this, std::placeholders::_1));

    static auto sub_navigation = this->create_subscription<std_msgs::msg::UInt8MultiArray>("navigation", 1,
                                                                                           std::bind(
                                                                                                   &MotionControlPure::Navigation,
                                                                                                   this,
                                                                                                   std::placeholders::_1));
    static auto sub_local_path = this->create_subscription<std_msgs::msg::UInt8MultiArray>("local_path", 1, std::bind(
            &MotionControlPure::LocalPath, this, std::placeholders::_1));
    // Corrected message type
}

void MotionControlPure::ReadYaml() {
    std::string pure = "pure.yaml";
    YAML::Node config = YAML::LoadFile(pure);
    // Process YAML data here
    mode = config["mode"].as<std::double_t>();
    tmp_v = config["V"].as<std::double_t>();
    tmp_a = config["A"].as<std::double_t>();
    tmp_th = config["Th"].as<std::double_t>();
    tmp_brake = config["Brake"].as<std::double_t>();
    wheel_base = config["wheel_base"].as<std::double_t>();
    tmp_max_steer = config["MaxSteer"].as<std::double_t>();
    test.limit_s = config["S"].as<std::double_t>();
    test.params.k1 = config["k1"].as<std::double_t>();
    test.a = config["a"].as<std::double_t>();
    test.params.k2 = config["k2"].as<std::double_t>();
    test.params.k3 = config["k3"].as<std::double_t>();
    t_v = config["t_v"].as<double>();
}
void MotionControlPure::SetV() {
    auto act_v = vs.v().Get(vs.v().size()-1);
    //auto act_v = my_parameter_;
    static bool setvalue = true, set_delay = true;
    double duration_seconds;
    auto dur_time_delay = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - end_time);
    end_time = std::chrono::high_resolution_clock::now();
    dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    duration_seconds = static_cast<double>(dur_time.count()) / 1000.0;
    if ( static_cast<double>(dur_time_delay.count()) / 1000.0 > 2 ) {
        //stable time
        duration_seconds = 0;
        std::cout << "it has been 2s which don't recieve local_path" << std::endl;
    }
    if ( duration_seconds > t_v && v_set == 0 && status_test && v_set01) {
        v_set = 1;
        stable_time = false;
        v_set01 = false;
    }
    else if ( duration_seconds > t_v && v_set == 1 && v_set12) {
        v_set = 2;
        stable_time = false;
        v_set12 = false;
    }
    else if ( duration_seconds > t_v && v_set == 2 && v_set23) {
        v_set = 2.5;
        stable_time = false;
        v_set23 = false;
    }
    else if ( duration_seconds > t_v && v_set == 2.5 && v_set32) {
        v_set = 2;
        stable_time = false;
        v_set32 = false;
    }
    else if ( duration_seconds > t_v && v_set == 2 && v_set21) {
        v_set = 1;
        stable_time = false;
        v_set21 = false;
    }
    else if ( duration_seconds > t_v && v_set == 1 && v_set10) {
        v_set = 0;
        stable_time = false;
        status_test = false;
        v_set10 = false;
    }
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_{};
    std::chrono::milliseconds dur_time_error1{};
    if ( setvalue == true ) {
        start_time_error = std::chrono::high_resolution_clock::now();
        start_time_ = std::chrono::high_resolution_clock::now();
        setvalue = false;
    }
    if ( fabs( v_set - act_v ) > 0.2 && fabs( v_set - act_v ) < 0.2 + 0.1) {
        start_time_error = std::chrono::high_resolution_clock::now();
    }
    if ( fabs( v_set - act_v ) < 0.2 - 0.1 ) {
        start_time_ = std::chrono::high_resolution_clock::now();
    }
    double dur_time_ = static_cast<double>(dur_time_error.count()) / 1000.0;
    //reset time
    if ( ( fabs( v_set - act_v ) < 0.2 && duration_seconds > t_v ) ||  dur_time_ > t_v / 6 || !stable_time) {
        start_time = std::chrono::high_resolution_clock::now();
        dur_time_error = std::chrono::milliseconds (0);
        stable_time = true;
    }
    if ( fabs( v_set - act_v ) > 0.2  && duration_seconds < t_v && stable_time) {
        dur_time_error1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_error);
        std::cout << " dur_time_error1  : " << static_cast<double>(dur_time_error1.count()) / 1000.0 << std::endl;
        auto dur_time_jud = std::chrono::duration_cast<std::chrono::milliseconds>(start_time_error - start_time_);
        if ( static_cast<double>(dur_time_jud.count()) / 1000.0 > 0 ) {
            dur_time_error = dur_time_error1 + dur_time_error;
        }
        auto time_since_epoch_end = end_time.time_since_epoch();
        auto time_since_epoch_start = start_time_error.time_since_epoch();
        auto duration_end_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch_end);
        auto duration_start_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch_start);
        std::cout << "End Time since epoch: " << duration_end_ms.count() << " milliseconds" << std::endl;
        std::cout << "Start Time since epoch: " << duration_start_ms.count() << " milliseconds" << std::endl;
    }
    //std::cout << " start_time  : " << start_time << std::endl;
    //std::cout << " start_time_error : "
    std::cout << " stable_time : " << stable_time << std::endl;
    //std::cout << " dur_time_error1  : " << static_cast<double>(dur_time_error1.count()) / 1000.0 << std::endl;
    std::cout << " duration_seconds : " <<  duration_seconds << std::endl;
    std::cout << " dur_time_error : " <<  static_cast<double>(dur_time_error.count()) / 1000.0 << std::endl;
}
