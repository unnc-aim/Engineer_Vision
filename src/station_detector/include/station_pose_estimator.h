#ifndef STATION_POSE_ESTIMATOR_H
#define STATION_POSE_ESTIMATOR_H

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <string>
#include <mutex>  // 添加互斥锁支持

#include "fusion_station_detector.h"
#include "kalman_filter.h"

class StationPoseEstimator : public rclcpp::Node {
public:
    explicit StationPoseEstimator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void initDetector();  // 新增：初始化检测器
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    // validateCorners函数已删除 - 现在使用FusionStationDetector::validateDetectedCorners
    geometry_msgs::msg::PoseStamped transformToBaseLink(
        const cv::Mat& rvec, const cv::Mat& tvec, 
        const std_msgs::msg::Header& header);
    void applyKalmanFilter(geometry_msgs::msg::PoseStamped& pose);
    void publishTransform(const geometry_msgs::msg::PoseStamped& pose);
    
    // 修改函数签名，添加 status_text 参数
    void publishDebugImage(const cv::Mat& frame, 
                          const std::vector<cv::Point2f>& corners,
                          const std::vector<std::vector<cv::Point>>& current_contours,
                          const std_msgs::msg::Header& header,
                          const std::string& status_text);
    
    // 新增函数：发布二值图像
    void publishBinaryImage(const cv::Mat& binary_image, 
                           const std_msgs::msg::Header& header);
    
    void initKalmanFilter();

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr binary_pub_;  // 新增二值图像发布者
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    std::unique_ptr<FusionStationDetector> detector_;
    bool detector_initialized_ = false;
    bool camera_info_received_ = false;
    bool debug_mode_ = false;
    bool show_binary_window_ = false;  // 新增：控制是否显示二值窗口
    std::vector<cv::Point2f> prev_corners_;
    
    std::vector<std::vector<cv::Point>> detected_contours_;

    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_;
    
    KalmanFilter kalman_filter_;

    // 添加互斥锁
    std::mutex image_mutex_;

    // 评估某个解的重投影误差（像素）
    double computeReprojError(const std::vector<cv::Point3f>& obj,
                          const std::vector<cv::Point2f>& img,
                          const cv::Mat& rvec, const cv::Mat& tvec) const;

// 朝向无关：在 4(或 8) 种角点排列里搜索最优解
    bool solvePnPOrientationAgnostic(const std::vector<cv::Point3f>& obj_pts,
                                 const std::vector<cv::Point2f>& img_corners,
                                 cv::Mat& best_rvec, cv::Mat& best_tvec,
                                 double& best_err,
                                 bool try_mirror = true);
    
    // 新增：几何启发式PnP求解辅助函数
    std::vector<std::array<int,4>> selectCandidateOrders(const std::vector<cv::Point2f>& image_pts);
    std::vector<std::array<int,4>> selectMirrorOrders(const std::vector<cv::Point2f>& image_pts);
    bool validateGeometricConsistency(const std::vector<cv::Point3f>& object_pts,
                                     const std::vector<cv::Point2f>& image_pts);
    std::vector<cv::Point2f> applyIndex(const std::vector<cv::Point2f>& pts, 
                                       const std::array<int,4>& idx);
};
#endif