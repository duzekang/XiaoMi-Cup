import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from aruco_interfaces.msg import ArucoMarkers
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import Pose

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # 参数声明
        self.declare_parameters(
            namespace='',
            parameters=[
                ('dictionary', 'DICT_4X4_50'),
                ('camera_topic', '/camera/image_raw'),
                ('marker_size', 0.05),  # 单位：米
                ('show_debug', True)
            ]
        )
        
        # 初始化Aruco字典
        self.aruco_dict = self._get_aruco_dict()
        self.detector_params = aruco.DetectorParameters()
        
        self.camera_matrix = np.array([
            [184.75, 0,     320],  # fx ≈ 184.75
            [0,     184.75, 240],  # fy ≈ 184.75
            [0,      0,      1 ]
        ], dtype=np.float32)

        self.dist_coeffs = np.array([
            [0.0],  # k1
            [0.0],  # k2
            [0.0],  # p1
            [0.0]   # p2
        ], dtype=np.float32)
        
        # 图像订阅
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self.image_callback,
            10)
        
        # 结果发布
        self.markers_pub = self.create_publisher(ArucoMarkers, '/aruco/markers', 10)
        self.debug_pub = self.create_publisher(Image, '/aruco/debug_image', 10)

    def _get_aruco_dict(self):
        dict_mapping = {
            'DICT_4X4_50': aruco.DICT_4X4_50,
            'DICT_5X5_50': aruco.DICT_5X5_50,
            'DICT_6X6_50': aruco.DICT_6X6_50
        }
        return aruco.getPredefinedDictionary(
            dict_mapping[self.get_parameter('dictionary').value])

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # 检测标记
            corners, ids, rejected = aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.detector_params)
            
            markers_msg = ArucoMarkers()
            markers_msg.header = msg.header
            
            if ids is not None:
                # 位姿估计
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners,
                    self.get_parameter('marker_size').value,
                    self.camera_matrix,
                    self.dist_coeffs)
                    
                from tf_transformations import quaternion_from_matrix
                
                # 填充消息
                for i in range(len(ids)):
                    # ID
                    markers_msg.ids.append(int(ids[i][0]))
                    
                    # 角点坐标（展开为列表）
                    markers_msg.corners.extend(
                        corners[i][0].flatten().tolist())
                    
                    # 位姿
                    pose = Pose()
                    pose.position.x = tvecs[i][0][0]
                    pose.position.y = tvecs[i][0][1]
                    pose.position.z = tvecs[i][0][2]
                    rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                    # 转换为四元数
                    # ... (需要实现旋转矩阵到四元数的转换)
                    markers_msg.poses.append(pose)
                
                # 绘制调试信息
                debug_image = aruco.drawDetectedMarkers(cv_image, corners, ids)
                for i in range(len(ids)):
                    cv2.drawFrameAxes(
                        debug_image, 
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvecs[i], 
                        tvecs[i],
                        0.1  # 轴长度
                    )
                
                # 发布调试图像
                if self.get_parameter('show_debug').value:
                    self.debug_pub.publish(
                        self.bridge.cv2_to_imgmsg(debug_image, 'bgr8'))
            
            # 发布检测结果
            self.markers_pub.publish(markers_msg)
            
        except Exception as e:
            self.get_logger().error(f'Processing failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    detector = ArucoDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()