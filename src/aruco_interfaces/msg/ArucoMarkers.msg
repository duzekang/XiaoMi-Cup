# 标准消息头（时间戳 + 坐标系）
std_msgs/Header header

# 检测到的标记ID列表
int32[] ids

# 每个标记的四个角点坐标（像素坐标系）
# 每个标记存储8个float32值：[x1,y1, x2,y2, x3,y3, x4,y4]
float32[] corners

# 可选位姿信息（需要相机标定参数）
geometry_msgs/Pose[] poses
