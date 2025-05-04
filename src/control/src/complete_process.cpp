#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <unistd.h>  // 提供 STDIN_FILENO 的定义
#include <rclcpp/rclcpp.hpp>
#include <lcm/lcm-cpp.hpp>
#include <yaml-cpp/yaml.h>
#include "../lcm_defines/lcm_types/robot_control_cmd_lcmt.hpp"
#include "../lcm_defines/lcm_types/simulator_lcmt.hpp"


enum State {
    INIT,
    STAND,
    MOVE,
    SIT,
    LINETRACK,
    SCANCODE,
    SCANARROW,
    STONEWAY,
    SLOPE,
    COMPLETED
};

struct Point {
    double x;  // X坐标
    double y;  // Y坐标
    Point() = default;  // 显式声明默认构造函数
    Point(double x_, double y_) : x(x_), y(y_) {}
};

class CompleteController : public rclcpp::Node {
    public:
        // 构造函数
        CompleteController() : Node("complete_process"),lcm("udpm://239.255.76.67:7671?ttl=255"),
                            lcm2("udpm://239.255.76.67:7667?ttl=255"),current_state(INIT) {
            // LCM初始化
                if (!lcm.good() || !lcm2.good()) {
                    RCLCPP_FATAL(this->get_logger(), "LCM初始化失败!");
                    rclcpp::shutdown();
                }
            // 参数声明
            this->declare_parameter("position_tolerance", 0.1);
            this->declare_parameter("sit_duration", 5.0);
            this->declare_parameter("x_coords", std::vector<double>{});
            this->declare_parameter("y_coords", std::vector<double>{});

            this->sit_duration = this->get_parameter("sit_duration").as_double();
            this->position_tolerance = this->get_parameter("position_tolerance").as_double();
            this->x_coords = this->get_parameter("x_coords").as_double_array();
            this->y_coords = this->get_parameter("y_coords").as_double_array();

            // 初始化ROS组件
            initialize_components();
            
            // 加载路径点
            load_points();
            
            // 创建ROS时间驱动的定时器
            control_timer = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&CompleteController::control_cycle, this));
        }

    private:
        // private成员变量
        double position_tolerance;  // 位置容差
        double sit_duration;
        lcm::LCM lcm;
        lcm::LCM lcm2;
        lcm_types::robot_control_cmd_lcmt cmd{};
        State current_state;
        size_t target_point= 0;

        std::vector<Point> points;
        std::vector<double> x_coords;
        std::vector<double> y_coords;
        Point current;
        double current_vx;
        double current_vy;
        double current_pitch;
        double distance = 0;
        bool flag_back = false;
        
        rclcpp::Time state_start_time;
        rclcpp::TimerBase::SharedPtr control_timer;


        // 初始化发布器和订阅器
        void initialize_components() {
            lcm2.subscribe<lcm_types::simulator_lcmt>("simulator_state", &CompleteController::handle_message, this);
        }

        //加载路径点
        void load_points() {
            try {
                RCLCPP_INFO(this->get_logger(), "x_coords 长度: %zu", x_coords.size());
                RCLCPP_INFO(this->get_logger(), "y_coords 长度: %zu", y_coords.size());
                if (x_coords.size() != y_coords.size()) {
                    throw std::runtime_error("坐标数组长度不匹配");
                }

                points.clear();
                for (size_t i = 0; i < x_coords.size(); ++i) {
                    points.emplace_back(x_coords[i], y_coords[i]);
                }

                RCLCPP_INFO(this->get_logger(), "成功加载 %zu 个路径点", points.size());
            } catch (const std::exception& e) {
                RCLCPP_FATAL(get_logger(), "参数加载失败: %s", e.what());
                throw std::runtime_error("Critical error: Failed to load points.");
            }
        }


        //处理订阅的位置和速度信息的回调函数
        void handle_message(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& chan, 
                        const lcm_types::simulator_lcmt* msg) {
            (void)rbuf; // 开发阶段显式标记
            (void)chan;
            // 更新全局位置信息
            current.x = msg->p[0];
            current.y = msg->p[1];
            current_vx = msg->vb[0];
            current_vy = msg->vb[1];
            current_pitch = msg->rpy[1];
        }


        // ------------------------------状态机主控制循环-----------------------------
        void control_cycle() {
            switch (current_state) {
                case INIT:
                {
                    RCLCPP_INFO(this->get_logger(), "系统初始化中...");
                    state_start_time = this->now();
                    cmd.life_count=0;
                    print_parameters();
                    current_state = STAND;
                    break;
                }

                case STAND:
                {
                    RCLCPP_INFO(this->get_logger(), "执行站立命令");
                    cmd.mode = 12;
                    cmd.gait_id = 0;
                    if ((this->now() - state_start_time).seconds() > 6.0) {
                        target_point = 0;
                        current_state = MOVE;
                    }
                    break;
                }

                case MOVE:
                {
                    //运动代码补充
                    if (target_point >= points.size()) {
                        current_state = COMPLETED;
                    }
                    if (check_position_reached()) {
                        RCLCPP_INFO(this->get_logger(), "到达目标点 %zu", target_point);
                        state_start_time = this->now();
                        current_state = SIT;
                    }
                    break;
                }

                case SIT:
                {
                    RCLCPP_INFO(this->get_logger(), "执行坐下命令");
                    cmd.mode = 7;
                    cmd.gait_id = 0;
                    if ((this->now() - state_start_time).seconds() > sit_duration) {
                        current_state = STAND;
                        state_start_time = this->now();
                    }
                    break;
                }

                case COMPLETED:
                {
                    RCLCPP_INFO(this->get_logger(), "任务完成");
                    break;
                }

                case STONEWAY:
                {
                    RCLCPP_INFO(this->get_logger(), "过石板路");
                    cmd.mode = 11;
                    cmd.gait_id = 26;
                    cmd.vel_des[0] = 0.6;

                    if(flag_back) cmd.vel_des[1] = -current.x * 0.5;
                    else cmd.vel_des[1] = current.x * 0.5;

                    if(check_position_reached()){
                        current_state=MOVE;
                        target_point++;
                    }
                    break;;
                }

                default:
                    RCLCPP_WARN(this->get_logger(), "未处理的状态: %d", current_state);
                    break;
            }

            cmd.life_count++;
            //发布指令
            lcm.publish("robot_control_cmd", &cmd);
        }
        //---------------------------------------------------------------------------


        //检测回车
        // bool check_for_enter_key() {
        //     struct timeval tv = {0L, 0L};
        //     fd_set fds;
        //     FD_ZERO(&fds);
        //     FD_SET(STDIN_FILENO, &fds); // STDIN_FILENO是标准输入的文件描述符

        //     int ret = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
        //     if (ret > 0) {
        //         // 检测到输入，读取并清空缓冲区
        //         std::string line;
        //         std::getline(std::cin, line);
        //         return true;
        //     }
        //     return false;
        // }


        //判断是否到点
        bool check_position_reached() {
            const auto& target = points[target_point];
            const double dx = current.x - target.x;
            const double dy = current.y - target.y;
            return std::hypot(dx, dy) < position_tolerance;
        }

        void print_parameters(){
            std::cout<<"position_tolerance :"<<position_tolerance<<"\n";
            std::cout<<"sit_duration :"<<sit_duration<<std::endl;
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CompleteController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}