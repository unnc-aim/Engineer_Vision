# Engineer Vision 

一个基于ROS2的智能视觉检测与位姿估计系统，专门用于检测和定位工程huankuang目标。
项目概述

本项目包含两个主要ROS2包：
`mindvision_camera`: 工业相机驱动包，支持海康威视MindVision系列相机
`station_detector`: 站点检测与位姿估计包，实现基于计算机视觉的目标检测和6DOF位姿估计

主要功能

相机功能 (`mindvision_camera`)
  支持海康威视MindVision工业相机
  自动相机标定和参数管理
  实时图像采集和发布
  相机参数动态配置

检测功能 (`station_detector`)
  目标检测: L形轮廓检测
  角点提取: 角点检测和排序
  位姿估计: 基于PnP算法的6DOF位姿估计
  卡尔曼滤波: 位姿数据平滑和预测
  调试可视化: 调试图像输出

项目结构


Engineer_Vision/
├── src/
│   ├── mindvision_camera/          # 相机驱动包
│   │   ├── src/                    # 源代码
│   │   ├── config/                 # 相机配置文件
│   │   ├── launch/                 # 启动文件
│   │   └── mvsdk/                  # MindVision SDK头文件
│   └── station_detector/           # 站点检测包
│       ├── src/                    # 源代码
│       ├── include/                # 头文件
│       ├── config/                 # 检测参数配置
│       ├── launch/                 # 启动文件
│       ├── scripts/                # 辅助脚本
│       └── test_images/            # 测试图像
├── build/                          # 编译输出
├── install/                        # 安装文件
├── log/                           # 日志文件
├── debug_images/                  # 调试图像输出
└── tools/                         # 工具脚本


1. 静态图像测试

启动静态图像发布和检测
ros2 launch station_detector static_image_pub.launch.py

查看位姿输出
ros2 topic echo /station_pose

查看调试图像
ros2 run image_tools showimage --ros-args -r image:=/debug_image


2. 实时相机检测（未测试）

启动相机和检测系统
ros2 launch station_detector real_camera.launch.py

查看检测结果
ros2 topic echo /station_pose
ros2 topic hz /station_pose

（未测试）
3. 视频文件测试
使用视频文件进行测试：

# 启动视频播放和检测
ros2 launch station_detector test_video.launch.py



位姿数据 (`/station_pose`)
  消息类型: `geometry_msgs/msg/PoseStamped`
  坐标系: `camera_optical_frame`
  发布频率: 5Hz
  数据内容:
    位置: x, y, z (米)
    方向: 四元数 (x, y, z, w)

调试图像
系统在`debug_images/`目录下生成以下调试图像：
`raw.png`: 原始输入图像
`mask.png`: 颜色分割掩码
`all_contours.png`: 所有检测到的轮廓
`filtered_contours.png`: 过滤后的轮廓
`corners.png`: 检测到的角点
`fitted_rectangle.png`: 拟合的矩形
`debug_all.png`: 综合调试信息



检测参数 (`config/station_params.yaml`)

detection_params:
  min_contour_area: 30.0        # 最小轮廓面积
  max_contour_area: 10000.0     # 最大轮廓面积
  min_contour_points: 20        # 最小轮廓点数

color_detection:
  red_lower_h: 0                # 红色HSV下限
  red_lower_s: 20
  red_lower_v: 30
  red_upper_h: 25
  red_upper_s: 255
  red_upper_v: 255
  #  更多颜色参数

estimation_params:
  max_reprojection_error: 5.0   # 最大重投影误差
  outer_size: 0.288             # 外框尺寸(米)
  inner_size: 0.240             # 内框尺寸(米)




 
 
 
 
 
 
 //
 调试与故障排除

1. **位姿输出为空**
   - 检查角点检测是否成功
   - 查看重投影误差是否超过阈值
   - 确认相机标定参数正确

2. **检测精度不佳**
   - 调整颜色检测阈值
   - 优化轮廓过滤参数
   - 检查光照条件



### 调试命令
```bash
# 查看节点状态
ros2 node list
ros2 node info /station_pose_estimator

# 查看话题信息
ros2 topic list
ros2 topic info /station_pose
ros2 topic hz /station_pose

# 查看参数
ros2 param list /station_pose_estimator
ros2 param get /station_pose_estimator debug




性能指标
测精度: 重投影误差 < 1.0像素
处理频率: 5Hz (可配置)
检测范围 0.5m - 5.0m
角度范围: ±45° (俯仰/偏航)
//