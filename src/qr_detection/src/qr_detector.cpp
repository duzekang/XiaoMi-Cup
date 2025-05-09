// #include <rclcpp/rclcpp.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/msg/image.hpp>
// #include <std_msgs/msg/string.hpp>
// #include <zbar.h>
// #include <opencv2/opencv.hpp>

// class QrDetector : public rclcpp::Node {
// public:
//     QrDetector() : Node("qr_detector") {
//         sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//             "RGB_camera/image_raw", 
//             rclcpp::SensorDataQoS(),
//             [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
//                 this->imageCallback(msg);
//             });

//         qr_pub_ = this->create_publisher<std_msgs::msg::String>("qr_code", 10);
//         scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_X_DENSITY, 1);
//         scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_Y_DENSITY, 1);
//     }

// private:
//     void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
//         try {
//             cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
//             cv::Mat gray_img;
//             cv::cvtColor(cv_ptr->image, gray_img, cv::COLOR_BGR2GRAY);

//             zbar::Image img(gray_img.cols, gray_img.rows, "Y800", 
//                            gray_img.data, gray_img.cols * gray_img.rows);
            
//             int scan_result = scanner_.scan(img);
//             if (scan_result < 0) {
//                 RCLCPP_ERROR(get_logger(), "ZBar扫描失败，错误码: %d", scan_result);
//                 return;
//             }
//             auto symbol=img.symbol_begin();
//             if(symbol == img.symbol_end()){
//                 RCLCPP_WARN(get_logger(), "weishibie");
//             }
//             for(symbol = img.symbol_begin(); symbol != img.symbol_end(); ++symbol) {
//                 std_msgs::msg::String qr_msg;
//                 qr_msg.data = symbol->get_data();
//                 qr_pub_->publish(qr_msg);
//                 RCLCPP_INFO(get_logger(), "Detected QR: %s", qr_msg.data.c_str());
//             }
//             cv::imshow("QR Detection", cv_ptr->image);
//             cv::waitKey(1);
//         } catch (const cv_bridge::Exception& e) {
//             RCLCPP_ERROR(get_logger(), "CV Bridge Error: %s", e.what());
//         }
//     }

//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_pub_;
//     zbar::ImageScanner scanner_;
// };

// int main(int argc, char** argv) {
//     setenv("NO_AT_BRIDGE", "1", 1);
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<QrDetector>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

#include "qr_detection/qr_detection.hpp"

QRCodeReader::QRCodeReader() : Node("qr_detecor")
{
    // 初始化ZBar扫描器
    scanner_ = std::make_unique<zbar::ImageScanner>();
    scanner_->set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    // 创建订阅者，订阅RGB摄像头图像话题
    auto qos = rclcpp::SensorDataQoS().keep_last(10).best_effort();
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "RGB_camera/image_raw", 
        qos, 
        std::bind(&QRCodeReader::imageCallback, this, std::placeholders::_1));

    // 创建发布者，发布识别到的二维码信息
    publisher_ = this->create_publisher<std_msgs::msg::String>("qr_code_info", 10);
    gray_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "gray_image", 
            10);
    RCLCPP_INFO(this->get_logger(), "QR Code Reader节点已启动，等待图像输入...");
}

void QRCodeReader::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        // 将ROS图像消息转换为OpenCV格式
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        // 转换为灰度图像
        cv::Mat gray;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

        // fa bu hui du tu xiang
        auto gray_msg = cv_bridge::CvImage(
            msg->header, 
            "mono8", 
            gray
        ).toImageMsg();
        gray_publisher_->publish(*gray_msg);
        
        
        // 将OpenCV图像转换为ZBar图像
        zbar::Image zbar_image(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);
        
        // 扫描二维码
        (void)scanner_->scan(zbar_image);

        
        // zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
        // if(symbol == zbar_image.symbol_end()){
        //     RCLCPP_WARN(get_logger(), "weishibie");
        // }

        // 遍历检测到的二维码
        for(zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); 
            symbol != zbar_image.symbol_end(); 
            ++symbol) {
            
            // 提取二维码边界框位置
            std::vector<cv::Point> points;
            for(int i = 0; i < symbol->get_location_size(); i++) {
                points.push_back(cv::Point(
                    symbol->get_location_x(i), 
                    symbol->get_location_y(i)));
            }
            
            // 绘制边界框
            cv::RotatedRect rect = cv::minAreaRect(points);
            cv::Point2f vertices[4];
            rect.points(vertices);
            for(int i = 0; i < 4; i++) {
                cv::line(cv_ptr->image, vertices[i], vertices[(i+1)%4], cv::Scalar(0, 0, 255), 2);
            }
            
            // 提取二维码数据
            std::string data = symbol->get_data();
            
            // 在图像上绘制二维码数据
            cv::putText(cv_ptr->image, data, 
                        cv::Point(10, 30), 
                        cv::FONT_HERSHEY_SIMPLEX, 
                        0.7, cv::Scalar(0, 0, 255), 2);
            
            // 发布二维码信息
            auto qr_msg = std_msgs::msg::String();
            qr_msg.data = data;
            publisher_->publish(qr_msg);
            
            RCLCPP_INFO(this->get_logger(), "检测到二维码: %s", data.c_str());
        }
        
        // 显示结果图像
        cv::imshow("QR Code Reader", cv_ptr->image);
        cv::waitKey(1);
        
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "处理图像时发生错误: %s", e.what());
    }
}

int main(int argc, char * argv[])
{
    setenv("NO_AT_BRIDGE", "1", 1);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QRCodeReader>());
    rclcpp::shutdown();
    return 0;
}
