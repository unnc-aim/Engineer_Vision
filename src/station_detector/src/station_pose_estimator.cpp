#include "station_pose_estimator.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <tf2/exceptions.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <memory>
#include <array>
#include <mutex>
#include <algorithm>
#include <numeric>
#include <limits>

// 辅助函数：确保图像连续存储
void ensureContinuousImage(cv::Mat& image) {
    if (!image.isContinuous()) {
        cv::Mat temp;
        image.copyTo(temp);
        image = temp;
    }
}

StationPoseEstimator::StationPoseEstimator(const rclcpp::NodeOptions& options)
: Node("station_pose_estimator", options),
  detector_initialized_(false),
  camera_info_received_(false),
  debug_mode_(false),
  show_binary_window_(false),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this)),
  tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
{
    // 参数声明 - 只声明本节点需要的参数
    declare_parameter("publish_tf", true);
    declare_parameter("max_reprojection_error", 2.0);
    declare_parameter("corner_filter_gain", 0.1);
    declare_parameter("show_binary_window", false);
    
    // debug参数由FusionStationDetector管理，这里先设为false
    debug_mode_ = false;
    show_binary_window_ = get_parameter("show_binary_window").as_bool();
    RCLCPP_INFO(get_logger(), "Debug mode: %s", debug_mode_ ? "ON" : "OFF");
    RCLCPP_INFO(get_logger(), "Show binary window: %s", show_binary_window_ ? "ON" : "OFF");

    // 先创建检测器但不初始化
    detector_ = std::make_unique<FusionStationDetector>();
    
    // 创建相机信息订阅 - 使用直接回调链
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", 10,
        std::bind(&StationPoseEstimator::cameraInfoCallback, this, std::placeholders::_1));

    initKalmanFilter();

    RCLCPP_INFO(this->get_logger(), "StationPoseEstimator initialized");
}

void StationPoseEstimator::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if(!camera_info_received_) {
        camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
        distortion_coeffs_ = cv::Mat(1, 5, CV_64F, const_cast<double*>(msg->d.data())).clone();
        camera_info_received_ = true;
        RCLCPP_INFO(get_logger(), "Received camera info");
        
        // 添加相机矩阵诊断信息
        RCLCPP_INFO(get_logger(), "Camera Matrix: [%f, %f, %f; %f, %f, %f; %f, %f, %f]",
                   camera_matrix_.at<double>(0,0), camera_matrix_.at<double>(0,1), camera_matrix_.at<double>(0,2),
                   camera_matrix_.at<double>(1,0), camera_matrix_.at<double>(1,1), camera_matrix_.at<double>(1,2),
                   camera_matrix_.at<double>(2,0), camera_matrix_.at<double>(2,1), camera_matrix_.at<double>(2,2));
        
        // 直接初始化检测器
        initDetector();
    }
}

void StationPoseEstimator::initDetector() {
    if (detector_initialized_) {
        return;
    }
    
    try {
        // 使用接收到的相机参数初始化检测器
        detector_->setCameraParameters(camera_matrix_, distortion_coeffs_);
        
        // 传递节点共享指针给检测器
        detector_->init(shared_from_this());
        
        // 现在才创建图像订阅
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received new image frame! Size: %dx%d", 
                           msg->width, msg->height);
                this->imageCallback(msg);
            });
        
        // 创建发布者   
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("station_pose", 10);
        debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug_image", 10);
        binary_pub_ = this->create_publisher<sensor_msgs::msg::Image>("binary_image", 10);
        
        RCLCPP_INFO(this->get_logger(), "FusionStationDetector initialized");
        
        // 从检测器获取debug模式设置
        debug_mode_ = detector_->isDebugMode();
        RCLCPP_INFO(this->get_logger(), "Debug mode from detector: %s", debug_mode_ ? "ON" : "OFF");
        
        // 检查检测器是否已校准
        if (!detector_->isCalibrated()) {
            RCLCPP_WARN(this->get_logger(), "Detector is not calibrated yet");
        } else {
            RCLCPP_INFO(this->get_logger(), "Detector is properly calibrated");
        }
        
        detector_initialized_ = true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Detector initialization failed: %s", e.what());
    }
}

void StationPoseEstimator::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    // 添加互斥锁防止多线程竞争
    std::lock_guard<std::mutex> lock(image_mutex_);
    
    RCLCPP_INFO(get_logger(), "开始处理图像: %dx%d", msg->width, msg->height);
    
    if (!detector_initialized_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000, 
            "Detector not initialized yet. Skipping frame.");
        publishDebugImage(cv_bridge::toCvCopy(msg, "bgr8")->image, {}, {}, msg->header, "Waiting for detector init");
        return;
    }

    if (!detector_->isCalibrated()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                            "Detector not calibrated yet. Skipping frame.");
        publishDebugImage(cv_bridge::toCvCopy(msg, "bgr8")->image, {}, {}, msg->header, "Not calibrated");
        return;
    }

    if(!camera_info_received_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for camera info...");
        publishDebugImage(cv_bridge::toCvCopy(msg, "bgr8")->image, {}, {}, msg->header, "No camera info");
        return;
    }
    
    if (camera_matrix_.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Camera matrix not initialized");
        publishDebugImage(cv_bridge::toCvCopy(msg, "bgr8")->image, {}, {}, msg->header, "Invalid camera matrix");
        return;
    }
    
    cv::Mat frame;
    try {
        frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    cv::Mat processed;
    try {
        processed = detector_->preprocess(frame);
        RCLCPP_DEBUG(get_logger(), "预处理完成");
        
        // 发布二值图像用于调试
        publishBinaryImage(processed, msg->header);
        
        // 可选：显示二值化窗口
        if (show_binary_window_) {
            cv::imshow("Binary Image", processed);
            cv::waitKey(1);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "预处理失败: %s", e.what());
        publishDebugImage(frame, {}, {}, msg->header, "Preprocessing failed");
        return;
    }
    
    std::vector<cv::Point2f> corners;
    std::vector<std::vector<cv::Point>> detected_contours;

    try {
        corners = detector_->detectCorners(processed, frame);
        RCLCPP_INFO(get_logger(), "检测到 %zu 个角点", corners.size());
        // 获取检测到的轮廓用于可视化
        detected_contours = detector_->getDetectedContours();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "角点检测失败: %s", e.what());
        corners = {};
        detected_contours.clear();
    }
    
    // 传递实际检测到的轮廓
    publishDebugImage(frame, corners, detected_contours, msg->header, 
                     corners.empty() ? "No corners" : "Processing");
    
    if(!prev_corners_.empty() && corners.size() == 4) {
        double gain = get_parameter("corner_filter_gain").as_double();
        for(int i=0; i<4; i++) {
            corners[i] = (1.0 - gain) * prev_corners_[i] + gain * corners[i];
        }
    }
    prev_corners_ = corners;
    
    if(corners.size() == 4) {
        RCLCPP_DEBUG(get_logger(), "Found 4 corners, validating...");
        
        // 使用检测器的验证函数，避免重复验证
        if(!detector_->validateDetectedCorners(corners)) {
            RCLCPP_INFO(get_logger(), "Corners validation failed");
            publishDebugImage(frame, corners, detected_contours, msg->header, "Validation failed");
            return;
        }
        
        RCLCPP_INFO(get_logger(), "Corners validation passed, proceeding to PnP solving");
        
        auto object_points = detector_->getObjectPoints();
        cv::Mat rvec, tvec;
        
        double best_err = std::numeric_limits<double>::infinity();

        try {
            bool ok = solvePnPOrientationAgnostic(object_points, corners, rvec, tvec, best_err, /*try_mirror=*/false);

            if (!ok) {
                RCLCPP_WARN(get_logger(), "Orientation-agnostic solvePnP failed");
                publishDebugImage(frame, corners, detected_contours, msg->header, "PnP failed");
                return;
            }

            RCLCPP_INFO(get_logger(), "Best reprojection error: %.2f (threshold: %.2f)", best_err, get_parameter("max_reprojection_error").as_double());

            if (best_err < get_parameter("max_reprojection_error").as_double()) {
                geometry_msgs::msg::PoseStamped pose_base =
                    transformToBaseLink(rvec, tvec, msg->header);

                if (!pose_base.header.frame_id.empty()) {
                    applyKalmanFilter(pose_base);
                    pose_pub_->publish(pose_base);
                    RCLCPP_INFO(get_logger(), "Published station pose in base_link frame");
                    publishTransform(pose_base);
                    publishDebugImage(frame, corners, detected_contours, msg->header, "Pose found");
                } else {
                    // TF变换失败时，直接发布相机坐标系下的位姿
                    geometry_msgs::msg::PoseStamped pose_cam;
                    pose_cam.header = msg->header;
                    
                    // 旋转向量转换为旋转矩阵
                    cv::Mat rotation_matrix;
                    cv::Rodrigues(rvec, rotation_matrix);
                    
                    // 转换为四元数
                    tf2::Matrix3x3 tf_rot(
                        rotation_matrix.at<double>(0,0), rotation_matrix.at<double>(0,1), rotation_matrix.at<double>(0,2),
                        rotation_matrix.at<double>(1,0), rotation_matrix.at<double>(1,1), rotation_matrix.at<double>(1,2),
                        rotation_matrix.at<double>(2,0), rotation_matrix.at<double>(2,1), rotation_matrix.at<double>(2,2));
                    
                    tf2::Quaternion quat;
                    tf_rot.getRotation(quat);
                    
                    pose_cam.pose.position.x = tvec.at<double>(0);
                    pose_cam.pose.position.y = tvec.at<double>(1);
                    pose_cam.pose.position.z = tvec.at<double>(2);
                    pose_cam.pose.orientation = tf2::toMsg(quat);
                    
                    applyKalmanFilter(pose_cam);
                    pose_pub_->publish(pose_cam);
                    RCLCPP_INFO(get_logger(), "Published station pose in camera frame (TF transform failed)");
                    publishTransform(pose_cam);
                    publishDebugImage(frame, corners, detected_contours, msg->header, "Pose found (camera frame)");
                }
            } else {
                RCLCPP_DEBUG(get_logger(), "Reprojection error too high: %.2f", best_err);
                publishDebugImage(frame, corners, detected_contours, msg->header,
                                "Reproj error: " + std::to_string(best_err));
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(get_logger(), "SolvePnP error: %s", e.what());
            publishDebugImage(frame, corners, detected_contours, msg->header, "SolvePnP error");
        }
    } else {
        publishDebugImage(frame, corners, detected_contours, msg->header, 
                         "Corners: " + std::to_string(corners.size()));
    }
}

// validateCorners函数已删除 - 现在使用FusionStationDetector::validateDetectedCorners

geometry_msgs::msg::PoseStamped StationPoseEstimator::transformToBaseLink(
    const cv::Mat& rvec, const cv::Mat& tvec, 
    const std_msgs::msg::Header& header)
{
    geometry_msgs::msg::PoseStamped pose_cam;
    pose_cam.header = header;
    
    // 旋转向量转换为旋转矩阵
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    
    // 转换为四元数
    tf2::Matrix3x3 tf_rot(
        rotation_matrix.at<double>(0,0), rotation_matrix.at<double>(0,1), rotation_matrix.at<double>(0,2),
        rotation_matrix.at<double>(1,0), rotation_matrix.at<double>(1,1), rotation_matrix.at<double>(1,2),
        rotation_matrix.at<double>(2,0), rotation_matrix.at<double>(2,1), rotation_matrix.at<double>(2,2));
    
    tf2::Quaternion quat;
    tf_rot.getRotation(quat);
    
    pose_cam.pose.position.x = tvec.at<double>(0);
    pose_cam.pose.position.y = tvec.at<double>(1);
    pose_cam.pose.position.z = tvec.at<double>(2);
    pose_cam.pose.orientation = tf2::toMsg(quat);
    
    // 转换到base_link坐标系
    geometry_msgs::msg::PoseStamped pose_base;
    try {
        auto transform = tf_buffer_->lookupTransform(
            "base_link", header.frame_id, header.stamp,
            tf2::durationFromSec(0.1));
            
        tf2::doTransform(pose_cam, pose_base, transform);
        pose_base.header.frame_id = "base_link";
        return pose_base;
    } 
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF error: %s", ex.what());
        return geometry_msgs::msg::PoseStamped(); // 返回空位姿
    }
}

void StationPoseEstimator::applyKalmanFilter(geometry_msgs::msg::PoseStamped& pose) {
    // 卡尔曼滤波预测
    kalman_filter_.predict();
    
    // 测量向量 [x, y, z]
    Eigen::Vector3d measurement(
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z);
    
    // 更新卡尔曼滤波器
    kalman_filter_.update(measurement);
    
    // 获取滤波后状态
    Eigen::VectorXd state = kalman_filter_.getState();
    
    // 更新位姿
    pose.pose.position.x = state[0];
    pose.pose.position.y = state[1];
    pose.pose.position.z = state[2];
}

void StationPoseEstimator::publishTransform(const geometry_msgs::msg::PoseStamped& pose) {
    if(get_parameter("publish_tf").as_bool()) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header = pose.header;
        tf.child_frame_id = "exemption_station";
        tf.transform.translation.x = pose.pose.position.x;
        tf.transform.translation.y = pose.pose.position.y;
        tf.transform.translation.z = pose.pose.position.z;
        tf.transform.rotation = pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
        RCLCPP_DEBUG(get_logger(), "Published TF transform");
    }
}

void StationPoseEstimator::initKalmanFilter() {
    // 状态向量: [x, y, z, vx, vy, vz]
    Eigen::MatrixXd F(6,6);  // 状态转移矩阵
    F << 1,0,0,0.05,0,0,   // 时间步长为0.05s
         0,1,0,0,0.05,0,
         0,0,1,0,0,0.05,
         0,0,0,1,0,0,
         0,0,0,0,1,0,
         0,0,0,0,0,1;
    
     Eigen::MatrixXd Q = 0.01 * Eigen::MatrixXd::Identity(6,6);  // 过程噪声
    
    Eigen::MatrixXd H(3,6);  // 观测矩阵 (只能观测位置)
    H << 1,0,0,0,0,0,
         0,1,0,0,0,0,
         0,0,1,0,0,0;
    
    Eigen::MatrixXd R = 0.05 * Eigen::MatrixXd::Identity(3,3);  // 观测噪声

    
    kalman_filter_.init(F, H, Q, R, Eigen::VectorXd::Zero(6));
    RCLCPP_INFO(get_logger(), "Kalman filter initialized");
}

void StationPoseEstimator::publishBinaryImage(const cv::Mat& binary_image, 
                                             const std_msgs::msg::Header& header) 
{
    // 添加安全检查和详细日志
    if (!binary_pub_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                            "Binary publisher not initialized");
        return;
    }
    // 检查图像有效性
    if (binary_image.empty() || binary_image.cols < 10 || binary_image.rows < 10) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                            "Binary image is empty or too small, skip publishing");
        return;
    }
    
    // 检查图像类型 (必须为单通道)
    cv::Mat img_to_publish;
    if (binary_image.type() != CV_8UC1) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                            "Converting binary image type: %d to CV_8UC1", 
                            binary_image.type());
        binary_image.convertTo(img_to_publish, CV_8UC1);
    } else {
        img_to_publish = binary_image;
    }

    try {
        // 创建独立的图像副本
        cv::Mat img_copy = img_to_publish.clone();
        // 确保图像连续存储
        ensureContinuousImage(img_copy);
        
        // 使用共享指针确保图像数据在发布期间保持有效
        auto cv_image = std::make_shared<cv_bridge::CvImage>();
        cv_image->header = header;
        cv_image->encoding = "mono8";
        cv_image->image = img_copy;
        
        auto binary_msg = cv_image->toImageMsg();
        binary_pub_->publish(*binary_msg);
        RCLCPP_DEBUG(get_logger(), "Published binary image");
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error publishing binary image: %s", e.what());
    }
}

void StationPoseEstimator::publishDebugImage(
    const cv::Mat& frame, 
    const std::vector<cv::Point2f>& corners,
    const std::vector<std::vector<cv::Point>>& current_contours,
    const std_msgs::msg::Header& header,
    const std::string& status_text)
{
    if (!debug_pub_) {
        RCLCPP_WARN(get_logger(), "Debug publisher not initialized, skipping debug image");
        return;
    }
    
    // 检查原始图像有效性
    if (frame.empty() || frame.cols < 10 || frame.rows < 10) {
        RCLCPP_WARN(get_logger(), "Input frame is empty or too small, skip publishing debug image");
        return;
    }
    
    // 创建调试图像（缩放大图像以提高性能）
    cv::Mat debug;
    double scale = 1.0;
    const int min_dimension = 50; // 最小有效尺寸
    
    if (frame.cols > 1280 || frame.rows > 1024) {
        scale = 0.5;
        cv::Size new_size(
            std::max(min_dimension, static_cast<int>(frame.cols * scale)),
            std::max(min_dimension, static_cast<int>(frame.rows * scale))
        );
        cv::resize(frame, debug, new_size);
    } else {
        debug = frame.clone();
    }
    
    // 检查缩放后图像有效性
    if (debug.empty() || debug.cols < min_dimension || debug.rows < min_dimension) {
        RCLCPP_WARN(get_logger(), "Resized debug image is invalid, skip publishing");
        return;
    }
    
    const int width = debug.cols;
    const int height = debug.rows;
    
    // 安全绘制状态文本
    if (!status_text.empty() && height > 40) {
        cv::putText(debug, status_text, cv::Point(20, 40), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
    }
    
    // 安全绘制轮廓（使用缩放因子）
    for (const auto& contour : current_contours) {
        if (contour.empty()) continue;
        
        // 创建缩放后的轮廓
        std::vector<cv::Point> scaled_contour;
        for (const auto& pt : contour) {
            int x = static_cast<int>(pt.x * scale);
            int y = static_cast<int>(pt.y * scale);
            
            // 确保坐标在图像范围内
            if (x >= 0 && x < width && y >= 0 && y < height) {
                scaled_contour.push_back(cv::Point(x, y));
            }
        }
        
        if (!scaled_contour.empty()) {
            std::vector<std::vector<cv::Point>> temp = {scaled_contour};
            cv::drawContours(debug, temp, -1, cv::Scalar(0, 255, 255), 2);
        }
    }
    
    // 安全绘制角点（使用缩放因子）
    for (size_t i = 0; i < corners.size(); ++i) {
        int x = static_cast<int>(corners[i].x * scale);
        int y = static_cast<int>(corners[i].y * scale);
        
        if (x >= 0 && x < width && y >= 0 && y < height) {
            cv::circle(debug, cv::Point(x, y), 8 * scale, cv::Scalar(0, 255, 0), 2);
            
            // 安全绘制文本
            if (y + 30 < height) {
                cv::putText(debug, std::to_string(i), cv::Point(x + 10, y + 10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7 * scale, cv::Scalar(0, 0, 255), 2);
            }
        }
    }
    
    // 绘制连接线
    if(corners.size() == 4) {
        for(int i=0; i<4; i++) {
            int x1 = static_cast<int>(corners[i].x * scale);
            int y1 = static_cast<int>(corners[i].y * scale);
            int x2 = static_cast<int>(corners[(i+1)%4].x * scale);
            int y2 = static_cast<int>(corners[(i+1)%4].y * scale);
            
            if (x1 >= 0 && x1 < width && y1 >= 0 && y1 < height &&
                x2 >= 0 && x2 < width && y2 >= 0 && y2 < height) {
                cv::line(debug, cv::Point(x1, y1), cv::Point(x2, y2), 
                         cv::Scalar(255,0,0), 2);
            }
        }
    }
    
    // 添加时间戳和帧ID
    if (height > 120) {
        cv::putText(debug, "Frame: " + header.frame_id, cv::Point(20, 80), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7 * scale, cv::Scalar(200, 200, 200), 1);
        
        // 添加当前时间
        auto now = this->now();
        cv::putText(debug, "Time: " + std::to_string(now.seconds()), cv::Point(20, 120), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7 * scale, cv::Scalar(200, 200, 200), 1);
    }
    
    // 添加预处理参数信息 - 避免访问可能不存在的参数
    // 使用try-catch确保不会崩溃
    try {
        // 只尝试获取本节点声明的参数
        double max_error = get_parameter("max_reprojection_error").as_double();
        double gain = get_parameter("corner_filter_gain").as_double();
        
        if (height > 160) {
            cv::putText(debug, "Max Error: " + std::to_string(max_error), 
                       cv::Point(20, 160), cv::FONT_HERSHEY_SIMPLEX, 0.5 * scale, cv::Scalar(200, 200, 200), 1);
        }
        if (height > 190) {
            cv::putText(debug, "Gain: " + std::to_string(gain), 
                       cv::Point(20, 190), cv::FONT_HERSHEY_SIMPLEX, 0.5 * scale, cv::Scalar(200, 200, 200), 1);
        }
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
        // 忽略未声明参数的错误
    } catch (...) {
        // 忽略其他错误
    }

    try {
        // 创建独立的图像副本
        cv::Mat debug_copy = debug.clone();
        // 确保图像连续存储
        ensureContinuousImage(debug_copy);
        
        // 使用共享指针确保图像数据在发布期间保持有效
        auto cv_image = std::make_shared<cv_bridge::CvImage>();
        cv_image->header = header;
        cv_image->encoding = "bgr8";
        cv_image->image = debug_copy;
        
        auto debug_msg = cv_image->toImageMsg();
        debug_pub_->publish(*debug_msg);
        RCLCPP_DEBUG(get_logger(), "Published debug image");
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error publishing debug image: %s", e.what());
    }
}

// 计算重投影误差的辅助函数
double StationPoseEstimator::computeReprojError(
    const std::vector<cv::Point3f>& obj,
    const std::vector<cv::Point2f>& img,
    const cv::Mat& rvec, const cv::Mat& tvec) const
{
    std::vector<cv::Point2f> projected;
    cv::projectPoints(obj, rvec, tvec, camera_matrix_, distortion_coeffs_, projected);

    double err_sum = 0.0;
    for (size_t i = 0; i < img.size() && i < projected.size(); ++i) {
        err_sum += cv::norm(projected[i] - img[i]);
    }
    return (img.empty() ? std::numeric_limits<double>::infinity() : err_sum / img.size());
}

// 优化版PnP求解 - 使用几何启发式减少尝试次数
bool StationPoseEstimator::solvePnPOrientationAgnostic(
    const std::vector<cv::Point3f>& object_pts,
    const std::vector<cv::Point2f>& image_pts,
    cv::Mat& best_rvec, cv::Mat& best_tvec,
    double& best_err,
    bool try_mirror)
{
    if (object_pts.size() != 4 || image_pts.size() != 4) {
        RCLCPP_WARN(get_logger(), "solvePnPOrientationAgnostic requires 4 object and 4 image points");
        return false;
    }

    // 1. 首先尝试检测器提供的顺序（通常最可能正确）
    std::vector<cv::Point2f> img_ordered = image_pts;
    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(object_pts, img_ordered, camera_matrix_, distortion_coeffs_,
                           rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    
    if (ok) {
        ok = cv::solvePnP(object_pts, img_ordered, camera_matrix_, distortion_coeffs_,
                          rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);
        if (ok) {
            double err = computeReprojError(object_pts, img_ordered, rvec, tvec);
            if (err < get_parameter("max_reprojection_error").as_double()) {
                best_err = err;
                best_rvec = rvec.clone();
                best_tvec = tvec.clone();
                RCLCPP_DEBUG(get_logger(), "Primary order succeeded with error: %.4f", err);
                return true;
            }
        }
    }

    // 2. 如果重投影误差大，使用几何启发式选择候选顺序
    std::vector<std::array<int,4>> candidates = selectCandidateOrders(image_pts);
    
    // 3. 如果允许镜像，添加镜像候选
    if (try_mirror) {
        auto mirror_candidates = selectMirrorOrders(image_pts);
        candidates.insert(candidates.end(), mirror_candidates.begin(), mirror_candidates.end());
    }

    best_err = std::numeric_limits<double>::infinity();
    bool found = false;

    for (const auto& idx : candidates) {
        std::vector<cv::Point2f> img = applyIndex(image_pts, idx);

        // 快速验证：检查几何一致性
        if (!validateGeometricConsistency(object_pts, img)) {
            continue;
        }

        // 求解PnP
        ok = cv::solvePnP(object_pts, img, camera_matrix_, distortion_coeffs_,
                          rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        if (!ok) continue;

        // 优化解
        ok = cv::solvePnP(object_pts, img, camera_matrix_, distortion_coeffs_,
                          rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);
        if (!ok) continue;

        double err = computeReprojError(object_pts, img, rvec, tvec);
        if (err < best_err) {
            best_err = err;
            best_rvec = rvec.clone();
            best_tvec = tvec.clone();
            found = true;
        }
    }

    if (!found) {
        RCLCPP_WARN(get_logger(), "solvePnPOrientationAgnostic: no valid pose found");
    } else {
        RCLCPP_DEBUG(get_logger(), "solvePnPOrientationAgnostic: best reproj err = %.4f", best_err);
    }
    return found;
}

// 新增：基于几何启发式选择候选顺序
std::vector<std::array<int,4>> StationPoseEstimator::selectCandidateOrders(
    const std::vector<cv::Point2f>& image_pts) {
    
    std::vector<std::array<int,4>> candidates;
    
    // 计算图像点的几何特征
    cv::Point2f center(0, 0);
    for (const auto& pt : image_pts) {
        center += pt;
    }
    center *= (1.0 / image_pts.size());
    
    // 按角度排序点
    std::vector<std::pair<double, int>> angle_index;
    for (size_t i = 0; i < image_pts.size(); ++i) {
        cv::Point2f vec = image_pts[i] - center;
        double angle = std::atan2(vec.y, vec.x);
        angle_index.push_back({angle, i});
    }
    std::sort(angle_index.begin(), angle_index.end());
    
    // 生成基于角度的候选顺序
    std::array<int,4> base_order;
    for (size_t i = 0; i < 4; ++i) {
        base_order[i] = angle_index[i].second;
    }
    
    // 尝试几种旋转
    candidates.push_back(base_order);
    candidates.push_back({base_order[1], base_order[2], base_order[3], base_order[0]});
    candidates.push_back({base_order[2], base_order[3], base_order[0], base_order[1]});
    candidates.push_back({base_order[3], base_order[0], base_order[1], base_order[2]});
    
    return candidates;
}

// 新增：选择镜像候选顺序
std::vector<std::array<int,4>> StationPoseEstimator::selectMirrorOrders(
    const std::vector<cv::Point2f>& image_pts) {
    
    std::vector<std::array<int,4>> mirror_candidates;
    
    // 镜像变换：交换x坐标
    std::vector<cv::Point2f> mirrored_pts = image_pts;
    for (auto& pt : mirrored_pts) {
        pt.x = -pt.x;  // 镜像变换
    }
    
    // 对镜像点应用相同的几何启发式
    auto candidates = selectCandidateOrders(mirrored_pts);
    mirror_candidates = candidates;
    
    return mirror_candidates;
}

// 新增：验证几何一致性
bool StationPoseEstimator::validateGeometricConsistency(
    const std::vector<cv::Point3f>& object_pts,
    const std::vector<cv::Point2f>& image_pts) {
    
    // 检查边长比例是否合理
    std::vector<double> object_lengths, image_lengths;
    
    for (int i = 0; i < 4; ++i) {
        int next = (i + 1) % 4;
        
        // 3D边长
        double obj_len = cv::norm(object_pts[next] - object_pts[i]);
        object_lengths.push_back(obj_len);
        
        // 2D边长
        double img_len = cv::norm(image_pts[next] - image_pts[i]);
        image_lengths.push_back(img_len);
    }
    
    // 检查边长比例是否在合理范围内
    for (size_t i = 0; i < object_lengths.size(); ++i) {
        if (object_lengths[i] > 1e-6 && image_lengths[i] > 1e-6) {
            double ratio = image_lengths[i] / object_lengths[i];
            if (ratio < 0.1 || ratio > 10.0) {  // 边长比例应在合理范围内
                return false;
            }
        }
    }
    
    return true;
}

// 新增：应用索引变换
std::vector<cv::Point2f> StationPoseEstimator::applyIndex(
    const std::vector<cv::Point2f>& pts, const std::array<int,4>& idx) {
    std::vector<cv::Point2f> out(4);
    for (int i = 0; i < 4; ++i) {
        out[i] = pts[idx[i]];
    }
    return out;
}