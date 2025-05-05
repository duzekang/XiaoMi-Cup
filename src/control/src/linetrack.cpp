#include "../lcm_defines/lcm_types/centroid_msg_lcmt.hpp" // 新增，形心消息类型
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <lcm/lcm-cpp.hpp>
#include "../lcm_defines/lcm_types/robot_control_cmd_lcmt.hpp"

// 需要你配置的参数（根据实际情况修改）
#define LINE_TRACK_TOPIC   "RGB_camera/image_raw"  // 图像话题
#define LCM_CONTROL_TOPIC "robot_control_cmd"      // LCM控制话题
#define BASE_VELOCITY     0.3                     // 基础前进速度(m/s)
const float k = 640.0f / 2.0f;      // 320 pixels/m
const float K_p = 1.2f;              // 中等响应速度
#define ANGULAR_GAIN (K_p / k)       // ≈ 0.00375

class LineTrackNode : public rclcpp::Node {

public:
    LineTrackNode() : Node("linetrack") {
        // LCM初始化（与原有系统一致）
        if (!lcm_.good()) {
            RCLCPP_FATAL(this->get_logger(), "LCM初始化失败!");
            rclcpp::shutdown();
        }

        // 订阅形心/偏移量消息（由Python端发布）
        lcm_.subscribe("centroid_topic", &LineTrackNode::onCentroidMsg, this);

        // LCM处理线程
        lcm_thread_ = std::thread([this]() {
            while (rclcpp::ok()) {
                lcm_.handleTimeout(50);
            }
        });
    }

    ~LineTrackNode() {
        if (lcm_thread_.joinable()) {
            lcm_thread_.join();
        }
    }

private:
    // 形心消息回调
    void onCentroidMsg(const lcm::ReceiveBuffer*, const std::string&, const lcm_types::centroid_msg_lcmt* msg) {
        // 直接用msg->error进行运动控制
        sendTrackingCommand(msg->error);
    }
    // void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    //     try {
    //         cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    //         cv::Mat processed = processImage(frame);
    //         publishProcessedImage(processed);
    //     } catch (const cv_bridge::Exception& e) {
    //         RCLCPP_ERROR(this->get_logger(), "CV桥接异常: %s", e.what());
    //     }
    // }

    // cv::Mat processImage(cv::Mat& input) {
    //     /* 需要你调整的关键参数 */
    //     const int roi_height = 320;                 // 🛠️ 裁剪高度（像素）
    //     const cv::Scalar lower_yellow(20, 100, 100); // 🛠️ HSV下限
    //     const cv::Scalar upper_yellow(30, 255, 255); // 🛠️ HSV上限

    //     // 裁剪ROI（取图像下半部分）
    //     cv::Mat roi = input(cv::Rect(0, input.rows - roi_height, input.cols, roi_height));

    //     // 颜色空间转换+掩膜生成
    //     cv::Mat hsv, mask;
    //     cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
    //     cv::inRange(hsv, lower_yellow, upper_yellow, mask);

    //     // 计算质心偏差
    //     cv::Moments m = cv::moments(mask, true);
    //     if (m.m00 > 1000) {  // 有效检测阈值（可调整）
    //         float cx = m.m10 / m.m00;
    //         float error = cx - (roi.cols / 2.0f);
    //         sendTrackingCommand(error);
    //     } else {
    //         sendStopCommand();
    //     }

    //     return mask;
 }

    void sendTrackingCommand(float lateral_error) {
        lcm_types::robot_control_cmd_lcmt cmd{};
        
        // === 核心修改点：不再设置 mode 字段 ===
        // 直接发送速度指令
        cmd.vel_des[0] = BASE_VELOCITY;          // X方向速度（前进）
        cmd.vel_des[2] = -lateral_error * ANGULAR_GAIN; // Yaw角速度（转向）
        
        // 其他字段保持默认值（根据你的需求可选）
        cmd.rpy_des[0] = 0.0f;  // roll保持水平
        cmd.rpy_des[1] = 0.0f;  // pitch保持水平
        
        lcm_.publish(LCM_CONTROL_TOPIC, &cmd);
    }

    void sendStopCommand() {
        lcm_types::robot_control_cmd_lcmt cmd{};
        // 将速度归零即可停止
        cmd.vel_des[0] = 0.0f;
        cmd.vel_des[2] = 0.0f;
        lcm_.publish(LCM_CONTROL_TOPIC, &cmd);
    }

//     void publishProcessedImage(const cv::Mat& mask) {
//         auto msg = cv_bridge::CvImage(
//             std_msgs::msg::Header(), 
//             "mono8", 
//             mask
//         ).toImageMsg();
//         processed_pub_.publish(msg);
//     }



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineTrackNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}