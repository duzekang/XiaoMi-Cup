#ifndef QR_CODE_READER_HPP
#define QR_CODE_READER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>

class QRCodeReader : public rclcpp::Node
{
public:
    QRCodeReader();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr gray_publisher_;
    std::unique_ptr<zbar::ImageScanner> scanner_;
};

#endif // QR_CODE_READER_HPP