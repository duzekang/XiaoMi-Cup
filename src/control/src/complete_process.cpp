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
#include <std_msgs/msg/string.hpp> 


#define slope_rad 20* M_PI/180.0


enum State {
    INIT,
    STAND,
    MOVE,
    SIT,
    LINETRACK,
    SCANCODE,
    SCANARROW,
    STONEWAY,
    ASCEND,
    DESCEND,
    LIMIT,
    YELLOW,
    COMPLETED
};

struct Point {
    double x;  // X坐标
    double y;  // Y坐标
    Point() = default;  // 显式声明默认构造函数
    Point(double x_, double y_) : x(x_), y(y_) {}
};

// 横向位置PID控制
class LateralController {
    public:
        LateralController(double kp) : kp(kp) {}
        double calculate(double current, double target) {
            return kp * (current - target);
        }
    private:
        double kp;
};
static LateralController lateral_controller(0.5);  // 比例系数

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

        std::string last_qr_message;
        int same_message_count = 0;
        bool scanned = false;
        std::string code_A;
        std::string code_B;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_subscription;

        // 初始化发布器和订阅器
        void initialize_components() {
            lcm2.subscribe<lcm_types::simulator_lcmt>("simulator_state", &CompleteController::handle_message, this);
            qr_subscription = this->create_subscription<std_msgs::msg::String>(
                "/qr_code_info",
                10,
                [this](const std_msgs::msg::String::SharedPtr msg) {
                    this->qr_callback(msg);
                });
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

        void qr_callback(const std_msgs::msg::String::SharedPtr msg) {
            // 连续相同消息检测
            if (msg->data == last_qr_message) {
                same_message_count++;
            } else {
                same_message_count = 0;
                last_qr_message = msg->data;
            }

            // 设置扫描完成标志
            scanned = (same_message_count >= 10);  // 连续10次相同视为有效扫描

            // 消息分类存储
            if (!msg->data.empty()) {
                if (msg->data[0] == 'A') {
                    code_A = msg->data;
                } else if (msg->data[0] == 'B') {
                    code_B = msg->data;
                }
            }

            RCLCPP_DEBUG(this->get_logger(), "Received QR: %s, Scanned: %d", 
                        msg->data.c_str(), scanned);
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

                case SCANCODE:
                {
                    cmd.mode = 3;
                    cmd.gait_id = 0;
                    cmd.pos_des[2]=0.23;
                    cmd.rpy_des[1]=-0.22;

                    // 当扫码成功后触发状态转移
                    if (scanned) {
                        RCLCPP_INFO(this->get_logger(), "Scanned code: A=%s B=%s", 
                                code_A.c_str(), code_B.c_str());
                        current_state = MOVE;
                        scanned = false;  // 重置标志
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
                        state_start_time = this->now();
                        current_state=MOVE;
                        target_point++;
                    }
                    break;;
                }

                case ASCEND: 
                {  // 上坡模式
                    double lateral_vel = lateral_controller.calculate(current.x, 2.0);
                    cmd.mode = 11;
                    cmd.gait_id = 26;
                    
                    cmd.vel_des[0] = 0.4;  // X方向速度
                    //cmd.vel_des[1] = lateral_vel;
                    cmd.vel_des[2] = 0.0;  // 禁止偏航旋转
                    
                    // 姿态补偿（最大允许0.52rad≈30度）
                    cmd.rpy_des[1] = -0.25; // 增加20%补偿量
                    cmd.rpy_des[2] = 1.523;
                    
                    // 高度控制（基于斜坡几何计算）
                    cmd.pos_des[2] = 0.28 * cos(slope_rad); // 实际高度≈0.26m
                    
                    // 质心前移补偿（不超过0.235m限制）
                    cmd.pos_des[0] = 0.2 * tan(slope_rad); // 约0.05m
                    
                    // 足端轨迹优化
                    cmd.foot_pose[0] = 0.04;  // 前腿X方向偏移（最大允许0.04m）
                    //cmd.foot_pose[3] = -0.02; // 后腿X方向回缩
                    
                    // 步态参数（基于TROT_24_16特性）
                    cmd.step_height[0] = 0.06; // 前腿最大步高（表格允许值）
                    cmd.step_height[1] = 0.04; // 后腿较低步高
                    
                    // 启用MPC轨迹跟踪
                    cmd.value = 0x01;
                    if (current.y >= 8.0) { // 到达Y=8米转下坡
                        current_state = DESCEND;
                        state_start_time = this->now();
                        RCLCPP_INFO(this->get_logger(), "到达Y=%.2fm，开始下坡", current.y);
                    }
                    break;
                }
                
                case DESCEND: 
                {  // 下坡模式
                    double lateral_vel = lateral_controller.calculate(current.x, 2.0);
                    cmd.mode = 11;
                    cmd.gait_id = 26;
                    cmd.contact = 0x0F;
                    
                    cmd.vel_des[0] = 0.4;           // 降低前进速度
                    cmd.vel_des[1] = lateral_vel * 1.2; // 加强位置修正
                    
                    cmd.rpy_des[1] = 0.30; // 增加20%补偿量
                    cmd.rpy_des[2] = 1.523;
                    
                    // 高度控制（基于斜坡几何计算）
                    cmd.pos_des[2] = 0.28 * cos(slope_rad);
                    
                    // 质心后移补偿
                    cmd.pos_des[0] = -0.2 * tan(slope_rad);
                    
                    // 足端轨迹优化
                    cmd.foot_pose[0] = -0.04;
                    
                    cmd.step_height[0] = 0.04; // 前腿最大步高（表格允许值）
                    cmd.step_height[1] = 0.06; // 后腿较低步高
                    
                    // 运动保护
                    cmd.value = 0x01;      // 启用MPC轨迹跟踪

                    if (current.y >= 10.5) { // 到达Y=10.5米转平地
                        current_state = MOVE;
                        state_start_time = this->now();
                        RCLCPP_INFO(this->get_logger(), "到达Y=%.2fm，开始平地", current.y);
                    }
                    break;
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