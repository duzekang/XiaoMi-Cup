import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
import math

class LineTrackNode(Node):
    def __init__(self):
        super().__init__('linetrack')
        self.subscription = self.create_subscription(
            Image,  # 订阅原始图像Image类型
            'RGB_camera/image_raw',  # 根据实际话题名调整
            self.image_callback, 
            qos_profile=rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                depth=5
            )
        )

        # 速度话题发布者 (QoS策略需配置)
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            'joy_teleop/cmd_vel',
            qos_profile=rclpy.qos.QoSProfile(depth=10))
        
        # 压缩图像发布者
        self.image_pub = self.create_publisher(
            Image,
            'processed_image',
            qos_profile=rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                depth=5
            )
        )
        
        self.cvBridge = CvBridge()

        # 新增：发布偏移量到C++ linetrack节点
        from std_msgs.msg import Float32
        self.error_pub = self.create_publisher(Float32, 'line_error', 10)

    def process_image(self, cv_image):
        # 图像裁剪（保持原始逻辑）
        height, width, _ = cv_image.shape
        descentre = 50
        rows_to_watch = 100
        #crop_img = cv_image[(height//4 + descentre) : (height//4 + descentre + rows_to_watch), 1:width]
        crop_img=cv_image
        # 颜色空间转换
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        # 设置黄色阈值范围
        lower_yellow = np.array([26, 43, 46])
        upper_yellow = np.array([34, 255, 255])

        # 生成掩模并提取黄色区域
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 计算中心偏差
        m = cv2.moments(mask, False)
        try:
            cx = m['m10']/m['m00']
            crop_width = crop_img.shape[1]
            error_x = cx - (crop_width // 2)  # 使用裁剪区域宽度
            print('cx:     ',cx)
            print('error_x:',error_x)
        except (ZeroDivisionError, KeyError):
            error_x = 0

        # 新增：发布偏移量到C++ linetrack节点
        from std_msgs.msg import Float32
        self.error_pub.publish(Float32(data=float(error_x)))

        # # 速度控制消息
        # twist_object = Twist()
        # # 设置固定的线速度值，单位 m/s
        # twist_object.linear.x = 0.05
        # # 将横向像素的偏差除以一个系数，该系数决定角速度值的大小，
        # # 需要根据线速度和图像总宽度进行调节，才能达到比较好的循迹效果
        # twist_object.angular.z = error_x / 80

        # # 条件判断（保持逻辑）
        # no_red = cv2.countNonZero(mask)
        # if (abs(twist_object.angular.z) < 10) and (no_red > 20000):
        #     self.cmd_vel_pub.publish(twist_object)
        #     self.get_logger().info(f"ANGULAR VALUE SENT ===> {twist_object.angular.z}")
        # else:
        #     twist_object.linear.x = 0.0
        #     twist_object.angular.z = 0.0
        #     self.cmd_vel_pub.publish(twist_object)
        #     self.get_logger().info("Out of range Stop!!!")

        # 图像发布（保持变量名）
        try:
            msg = Image()
            msg = self.cvBridge.cv2_to_imgmsg(mask, encoding="mono8")
            msg.header.stamp = self.get_clock().now().to_msg()
            self.image_pub.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error(str(e))

    def image_callback(self, msg):
        try:
            cv_image = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 执行后续处理（保持原处理流程）
            self.process_image(cv_image)
            
        except CvBridgeError as e:
            self.get_logger().error(f"图像转换失败: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = LineTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()