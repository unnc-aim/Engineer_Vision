#pragma once
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vector>
#include <memory>

// 颜色检测参数结构体
struct ColorDetectionParams {
    // 红色检测参数
    cv::Scalar red_lower;
    cv::Scalar red_upper;
    cv::Scalar red_upper_lower;
    cv::Scalar red_upper_upper;
    
    // 蓝色检测参数
    cv::Scalar blue_lower;
    cv::Scalar blue_upper;
    
    // 形态学处理参数
    int morphology_kernel_size;
    bool use_adaptive_kernel;
    
    // 默认构造函数
    ColorDetectionParams() {
        // 设置默认值
        red_lower = cv::Scalar(0, 30, 30);
        red_upper = cv::Scalar(25, 255, 255);
        red_upper_lower = cv::Scalar(155, 30, 50);
        red_upper_upper = cv::Scalar(180, 255, 255);
        blue_lower = cv::Scalar(85, 30, 30);
        blue_upper = cv::Scalar(144, 255, 255);
        morphology_kernel_size = 5;
        use_adaptive_kernel = true;
    }
};

class FusionStationDetector {
public:
    explicit FusionStationDetector();

    void setCameraParameters(const cv::Mat& camera_matrix, const cv::Mat& distortion_coeffs);

    // 统一为 shared_ptr，避免生命周期问题
    void init(std::shared_ptr<rclcpp::Node> node);

    // loadCalibration函数已删除 - 相机参数现在由StationPoseEstimator统一管理

    // 颜色分割（红+蓝并集）+ 形态学
    cv::Mat preprocess(const cv::Mat& input_image);

    // 从二值图中检测四个角（放宽为>=3也继续）- 新增原始图像参数用于调试
    std::vector<cv::Point2f> detectCorners(const cv::Mat& bin_image, const cv::Mat& original_image = cv::Mat());
    
    // 新增：验证检测到的角点质量
    bool validateDetectedCorners(const std::vector<cv::Point2f>& corners);
    
    // 新增：加载颜色检测参数
    void loadColorDetectionParams();
    
    // 新增：调试图像输出函数
    void saveDebugImage(const std::string& filename, const cv::Mat& image);
    void drawContours(const cv::Mat& original, const std::vector<std::vector<cv::Point>>& contours, const std::string& filename);
    void drawCorners(const cv::Mat& original, const std::vector<cv::Point2f>& corners, const std::string& filename);
    void drawFittedRectangle(const cv::Mat& original, const std::vector<cv::Point2f>& corners, const std::string& filename);
    void drawAllDebugInfo(const cv::Mat& original, const std::vector<std::vector<cv::Point>>& contours, 
                         const std::vector<cv::Point2f>& corners, const std::string& filename);

    const cv::Mat& getCameraMatrix() const { return camera_matrix_; }
    const cv::Mat& getDistortionCoeffs() const { return distortion_coeffs_; }
    std::vector<cv::Point3f> getObjectPoints() const;

    bool isCalibrated() const { return calibration_initialized_; }
    bool isDebugMode() const { return debug_mode_; }

    // 调试：返回最近一次检测到的轮廓
    const std::vector<std::vector<cv::Point>>& getDetectedContours() const { return detected_contours_; }

    // 单个"L"轮廓求拐角
    cv::Point2f findLShapedCorner(const std::vector<cv::Point>& contour);

private:
    std::vector<std::vector<cv::Point>> filterContours(const std::vector<std::vector<cv::Point>>& contours);
    std::vector<cv::Point2f> sortCorners(std::vector<cv::Point2f> corners);

    std::shared_ptr<rclcpp::Node> node_{nullptr};
    bool calibration_initialized_{false};
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_;

    std::vector<std::vector<cv::Point>> detected_contours_;
    
    bool debug_mode_;
    std::string debug_dir_;  // 调试图片保存目录
    
    // 颜色检测参数
    ColorDetectionParams color_params_;
};