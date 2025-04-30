import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class ArrowDetector(Node):
    def __init__(self):
        super().__init__('arrow_detector')
        self.bridge = CvBridge()
        
        # 创建图像订阅者
        self.subscription = self.create_subscription(
            Image,
            '/RGB_camera/image_raw',
            self.image_callback,
            qos_profile=rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                depth=5
            ))
        self.subscription  # 防止未使用变量警告

        # 创建方向发布者
        self.direction_publisher = self.create_publisher(
            String, 
            '/arrow_direction', 
            qos_profile=rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                depth=5
            )
        )
        
        # 初始化方向状态
        self.last_direction = None
        self.stable_counter = 0

    def preprocess(self, img):
        # 转换为HSV颜色空间
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # 设定绿色阈值范围（需根据实际颜色调试）
        lower_green = np.array([35, 50, 50])    # H:35-85对应广域绿色
        upper_green = np.array([85, 255, 255])
        
        # 创建颜色掩膜
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # 形态学操作优化（先闭运算填充空洞，后开运算去除噪点）
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # 保留颜色检测结果
        return mask

    def find_tip(self, points, convex_hull):
        length = len(points)
        indices = np.setdiff1d(range(length), convex_hull)
        for i in range(2):
            j = indices[i] + 2
            if j > length - 1:
                j = length - j
            p = j + 2
            if p > length - 1:
                p = length - p
            if np.all(points[j] == points[indices[i-1]-2]):
                return (tuple(points[j]), tuple(points[p]))
        return None

    def rotate(self, angle, xy):
        rotatex = math.cos(angle)*xy[0] - math.sin(angle)*xy[1]
        rotatey = math.cos(angle)*xy[0] + math.sin(angle)*xy[1]
        return (rotatex, rotatey)

    def image_callback(self, msg):
        try:
            # 转换ROS图像消息为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        # 预处理图像
        processed = self.preprocess(cv_image)
        
        # 寻找轮廓
        contours, _ = cv2.findContours(processed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        for cnt in contours:
            # 轮廓处理逻辑
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.025 * peri, True)
            hull = cv2.convexHull(approx, returnPoints=False)
            
            if hull is not None and len(hull) > 3:
                sides = len(hull)
                if 6 > sides > 3 and sides + 2 == len(approx):
                    points = approx[:,0,:]
                    try:
                        hull = hull.squeeze()
                        arrow_tip = self.find_tip(points, hull)
                        if arrow_tip:
                            # 计算方向
                            arrow_dir = np.array(arrow_tip[0]) - np.array(arrow_tip[1])
                            arrow_rotate = self.rotate(math.pi/4, arrow_dir)
                            
                            # 判断方向
                            if arrow_rotate[0] > 0:
                                direction = "向右" if arrow_rotate[1] > 0 else "向上"
                            else:
                                direction = "向下" if arrow_rotate[1] > 0 else "向左"
                            
                            self.get_logger().info(f'检测到箭头方向: {direction}')

                            if direction != self.last_direction:
                                self.stable_counter = 0
                                self.last_direction = direction
                            else:
                                self.stable_counter += 1
                            
                            # 连续5次相同结果才发布（防抖动）
                            if self.stable_counter == 5:  
                                msg = String()
                                msg.data = direction
                                self.direction_publisher.publish(msg)
                                self.get_logger().info(f'发布方向: {direction}')
                                self.stable_counter = 0  # 重置计数器
                    except Exception as e:
                        self.get_logger().warn(f'箭头检测异常: {e}')

def main(args=None):
    rclpy.init(args=args)
    detector = ArrowDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()