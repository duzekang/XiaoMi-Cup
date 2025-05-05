

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>



#define BASE_VELOCITY 0.3
const float k = 640.0f / 2.0f;
const float K_p = 1.2f;
#define ANGULAR_GAIN (K_p / k)

class LineTrackNode : public rclcpp::Node {
public:
    LineTrackNode() : Node("linetrack") {
        // 订阅偏移量话题
        error_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "line_error", 10,
            std::bind(&LineTrackNode::onErrorMsg, this, std::placeholders::_1)
        );
    }
private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr error_sub_;
    void onErrorMsg(const std_msgs::msg::Float32::SharedPtr msg) {
        sendTrackingCommand(msg->data);
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
        // TODO: 这里请根据你的实际底盘控制方式，发布ROS2控制指令（如geometry_msgs/msg/Twist等）
        // 示例：RCLCPP_INFO(this->get_logger(), "error=%.2f", lateral_error);
    }
    void sendStopCommand() {
        // TODO: 停止控制指令（如有需要）
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