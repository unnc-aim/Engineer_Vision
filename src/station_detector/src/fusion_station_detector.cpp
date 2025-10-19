#include "fusion_station_detector.h"
#include <vector>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>  
#include <ament_index_cpp/get_package_share_directory.hpp>  
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <future>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <unistd.h>  // for getcwd

FusionStationDetector::FusionStationDetector() 
    : node_(nullptr), calibration_initialized_(false), debug_mode_(false) {
}

void FusionStationDetector::init(std::shared_ptr<rclcpp::Node> node) {
    if (!node) {
        throw std::invalid_argument("Node pointer cannot be null");
    }

    // 确保只初始化一次
    if (node_) {
        RCLCPP_WARN(node->get_logger(), "Detector already initialized. Skipping re-initialization.");
        return;
    }
    
    node_ = node;
    
    // 只声明检测相关的参数，相机参数由外部设置
    node_->declare_parameter("station_width", 0.262);
    node_->declare_parameter("station_height", 0.262);
    node_->declare_parameter("min_contour_area", 400.0);
    node_->declare_parameter("max_contour_area", 12000.0);
    node_->declare_parameter("outer_size", 0.288);  // 外框尺寸288mm
    node_->declare_parameter("inner_size", 0.240);  // 内框尺寸240mm
    node_->declare_parameter("debug", false);  // 新增调试模式参数
    
    // 声明颜色检测参数
    node_->declare_parameter("color_detection.red_lower_h", 0);
    node_->declare_parameter("color_detection.red_lower_s", 30);
    node_->declare_parameter("color_detection.red_lower_v", 30);
    node_->declare_parameter("color_detection.red_upper_h", 25);
    node_->declare_parameter("color_detection.red_upper_s", 255);
    node_->declare_parameter("color_detection.red_upper_v", 255);
    node_->declare_parameter("color_detection.red_upper_lower_h", 155);
    node_->declare_parameter("color_detection.red_upper_lower_s", 30);
    node_->declare_parameter("color_detection.red_upper_lower_v", 50);
    node_->declare_parameter("color_detection.red_upper_upper_h", 180);
    node_->declare_parameter("color_detection.red_upper_upper_s", 255);
    node_->declare_parameter("color_detection.red_upper_upper_v", 255);
    node_->declare_parameter("color_detection.blue_lower_h", 85);
    node_->declare_parameter("color_detection.blue_lower_s", 30);
    node_->declare_parameter("color_detection.blue_lower_v", 30);
    node_->declare_parameter("color_detection.blue_upper_h", 144);
    node_->declare_parameter("color_detection.blue_upper_s", 255);
    node_->declare_parameter("color_detection.blue_upper_v", 255);
    node_->declare_parameter("color_detection.morphology_kernel_size", 5);
    node_->declare_parameter("color_detection.use_adaptive_kernel", true);
    
    // 获取调试模式参数
    debug_mode_ = node_->get_parameter("debug").as_bool();
    if (debug_mode_) {
        // 获取当前工作目录，在其下创建debug_images文件夹
        char* cwd = getcwd(nullptr, 0);
        if (cwd) {
            debug_dir_ = std::string(cwd) + "/debug_images";
            free(cwd);
        } else {
            // 如果获取工作目录失败，使用相对路径
            debug_dir_ = "debug_images";
        }
        
        RCLCPP_INFO(node_->get_logger(), "Debug mode enabled - intermediate images will be saved to %s", debug_dir_.c_str());
        
        // 提前创建调试目录
        if (std::filesystem::create_directories(debug_dir_)) {
            RCLCPP_INFO(node_->get_logger(), "Created debug directory: %s", debug_dir_.c_str());
        }
    }
    
    // 加载颜色检测参数
    loadColorDetectionParams();
}

void FusionStationDetector::setCameraParameters(const cv::Mat& camera_matrix, const cv::Mat& distortion_coeffs) {
    camera_matrix_ = camera_matrix.clone();
    distortion_coeffs_ = distortion_coeffs.clone();
    calibration_initialized_ = true;
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "Camera parameters set from external source");
    }
}

// loadCalibration函数已删除 - 相机参数现在由StationPoseEstimator统一管理

cv::Mat FusionStationDetector::preprocess(const cv::Mat& input_image) {
    if (camera_matrix_.empty() || distortion_coeffs_.empty()) {
        throw std::runtime_error("Camera parameters not loaded");
    }

    // === Debug: 保存原始图像 ===
    if (debug_mode_) {
        std::filesystem::create_directories(debug_dir_);
        cv::imwrite(debug_dir_ + "/raw.png", input_image);
        RCLCPP_INFO(node_->get_logger(), "Saved raw image to: %s/raw.png", debug_dir_.c_str());
    }

    cv::Mat undistorted;
    cv::undistort(input_image, undistorted, camera_matrix_, distortion_coeffs_);

    cv::Mat hsv, mask_red_lower, mask_red_upper, mask_red, mask_blue, combined;
    cv::cvtColor(undistorted, hsv, cv::COLOR_BGR2HSV);
    
    // 使用参数化的颜色检测
    // 红色下段
    cv::inRange(hsv, color_params_.red_lower, color_params_.red_upper, mask_red_lower);
    // 红色上段
    cv::inRange(hsv, color_params_.red_upper_lower, color_params_.red_upper_upper, mask_red_upper);
    cv::bitwise_or(mask_red_lower, mask_red_upper, mask_red);
    
    // 蓝色检测
    cv::inRange(hsv, color_params_.blue_lower, color_params_.blue_upper, mask_blue);
    cv::bitwise_or(mask_red, mask_blue, combined);

    // 形态学处理 - 使用参数化设置
    int k;
    if (color_params_.use_adaptive_kernel) {
        // 自适应卷积核：分辨率越大，核略大
        k = std::max(3, int(std::min(combined.cols, combined.rows) / 320.0 * color_params_.morphology_kernel_size));
    } else {
        // 使用固定核大小
        k = color_params_.morphology_kernel_size;
    }
    
    if (k < 3) k = 3;  // 最小核大小
    if ((k & 1) == 0) k += 1; // 确保为奇数
    
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    cv::morphologyEx(combined, combined, cv::MORPH_OPEN,  kernel);
    cv::morphologyEx(combined, combined, cv::MORPH_CLOSE, kernel);

    // === Debug: 保存掩膜图像 ===
    if (debug_mode_) {
        cv::imwrite(debug_dir_ + "/mask.png", combined);
        RCLCPP_INFO(node_->get_logger(), "Saved mask image to: %s/mask.png", debug_dir_.c_str());
    }

    return combined;
}

std::vector<cv::Point2f> FusionStationDetector::detectCorners(const cv::Mat& bin_image, const cv::Mat& original_image) {
    // 清空之前检测到的轮廓
    detected_contours_.clear();
    
    if (bin_image.empty() || bin_image.cols < 10 || bin_image.rows < 10) {
        if (node_) RCLCPP_WARN(node_->get_logger(), "Binary image is empty or too small");
        return {};
    }
    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        if (node_) RCLCPP_WARN(node_->get_logger(), "No contours found");
        return {};
    }

    // === Debug: 保存原始轮廓图像 ===
    if (debug_mode_ && !original_image.empty()) {
        std::filesystem::create_directories(debug_dir_);
        cv::Mat contour_debug = original_image.clone();
        cv::drawContours(contour_debug, contours, -1, cv::Scalar(0, 255, 0), 2);
        cv::imwrite(debug_dir_ + "/all_contours.png", contour_debug);
        RCLCPP_INFO(node_->get_logger(), "Saved all contours image to: %s/all_contours.png", debug_dir_.c_str());
    }

    auto filtered = filterContours(contours);
    detected_contours_ = filtered; // 存储检测到的轮廓
    
    // === Debug: 保存过滤后的轮廓图像 ===
    if (debug_mode_ && !original_image.empty()) {
        drawContours(original_image, filtered, "filtered_contours.png");
    }
    
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "Found %zu contours, filtered to %zu", 
                   contours.size(), filtered.size());
    }

    // 即使轮廓数量不足，也尝试检测角点并生成调试图像
    std::vector<cv::Point2f> corners;
    if (!filtered.empty()) {
        for(const auto& contour : filtered) {
            cv::Point2f corner = findLShapedCorner(contour);
            corners.push_back(corner);
        }
        
        // === Debug: 保存角点图像（即使角点数量不足） ===
        if (debug_mode_ && !original_image.empty() && !corners.empty()) {
            RCLCPP_INFO(node_->get_logger(), "Generating corner debug images for %zu corners", corners.size());
            drawCorners(original_image, corners, "corners.png");
            
            // 绘制所有调试信息
            drawAllDebugInfo(original_image, filtered, corners, "debug_all.png");
        } else {
            if (debug_mode_) {
                RCLCPP_INFO(node_->get_logger(), "Skipping corner debug images: debug_mode=%d, original_empty=%d, corners_empty=%d", 
                           debug_mode_, original_image.empty(), corners.empty());
            }
        }
    }

    if (filtered.size() < 3) {
        detected_contours_.clear();
        if (node_) RCLCPP_WARN(node_->get_logger(), "Too few contours: %zu", filtered.size());
        return corners; // 返回检测到的角点，即使数量不足
    }
    if (filtered.size() > 4) {
        filtered.resize(4); // 先保留前4个（已按面积/形状过滤过）
    }
    detected_contours_ = filtered;
    
    // 重新检测角点（因为可能被resize了）
    corners.clear();
    for(const auto& contour : filtered) {
        cv::Point2f corner = findLShapedCorner(contour);
        corners.push_back(corner);
    }
    
    auto sorted_corners = sortCorners(corners);

    // === Debug: 保存完整的角点图像（4个角点时） ===
    if (debug_mode_ && !original_image.empty() && sorted_corners.size() == 4) {
        // 绘制拟合的矩形
        drawFittedRectangle(original_image, sorted_corners, "fitted_rectangle.png");
        
        // 重新绘制完整的调试信息
        drawAllDebugInfo(original_image, filtered, sorted_corners, "debug_all.png");
    }

    return sorted_corners;
}

bool FusionStationDetector::validateDetectedCorners(const std::vector<cv::Point2f>& corners) {
    if (corners.size() != 4) {
        return false;
    }
    
    // 1. 内角校验 - 检查四个角的内角是否合理
    for(int i = 0; i < 4; i++) {
        cv::Point2f a = corners[i];
        cv::Point2f b = corners[(i+1)%4];
        cv::Point2f c = corners[(i+2)%4];
        
        cv::Point2f v1 = b - a;
        cv::Point2f v2 = b - c;
        
        double dot = v1.dot(v2);
        double len1 = cv::norm(v1);
        double len2 = cv::norm(v2);
        
        if(len1 > 0 && len2 > 0) {
            double angle = std::acos(dot / (len1 * len2)) * 180.0 / CV_PI;
            // 内角应该在40-130度之间
            if(angle < 40 || angle > 130) {
                if (node_) RCLCPP_INFO(node_->get_logger(), "Invalid angle: %.1f", angle);
                return false;
            }
        }
    }
    
    // 2. 面积校验 - 检查四边形面积是否合理
    double area = cv::contourArea(corners);
    if(area < 200) {
        if (node_) RCLCPP_INFO(node_->get_logger(), "Area too small: %.1f", area);
        return false;
    }
    
    // 3. 边长比例校验 - 检查相邻边长比例是否合理
    std::vector<double> side_lengths;
    for(int i = 0; i < 4; i++) {
        cv::Point2f p1 = corners[i];
        cv::Point2f p2 = corners[(i+1)%4];
        side_lengths.push_back(cv::norm(p2 - p1));
    }
    
    // 检查边长比例是否在合理范围内（避免过于细长的四边形）
    for(int i = 0; i < 4; i++) {
        double ratio = side_lengths[i] / side_lengths[(i+1)%4];
        if(ratio > 5.0 || ratio < 0.2) {  // 边长比例不应超过5:1
            if (node_) RCLCPP_INFO(node_->get_logger(), "Invalid side ratio: %.2f", ratio);
            return false;
        }
    }
    
    // 4. 凸性校验 - 确保四边形是凸的
    std::vector<cv::Point2f> hull;
    cv::convexHull(corners, hull);
    if(hull.size() != 4) {
        if (node_) RCLCPP_INFO(node_->get_logger(), "Not convex quadrilateral");
        return false;
    }
    
    return true;
}

void FusionStationDetector::loadColorDetectionParams() {
    if (!node_) {
        RCLCPP_WARN(node_->get_logger(), "Node pointer is null, using default color parameters");
        return;
    }
    
    try {
        // 加载红色检测参数
        color_params_.red_lower = cv::Scalar(
            node_->get_parameter("color_detection.red_lower_h").as_int(),
            node_->get_parameter("color_detection.red_lower_s").as_int(),
            node_->get_parameter("color_detection.red_lower_v").as_int()
        );
        color_params_.red_upper = cv::Scalar(
            node_->get_parameter("color_detection.red_upper_h").as_int(),
            node_->get_parameter("color_detection.red_upper_s").as_int(),
            node_->get_parameter("color_detection.red_upper_v").as_int()
        );
        color_params_.red_upper_lower = cv::Scalar(
            node_->get_parameter("color_detection.red_upper_lower_h").as_int(),
            node_->get_parameter("color_detection.red_upper_lower_s").as_int(),
            node_->get_parameter("color_detection.red_upper_lower_v").as_int()
        );
        color_params_.red_upper_upper = cv::Scalar(
            node_->get_parameter("color_detection.red_upper_upper_h").as_int(),
            node_->get_parameter("color_detection.red_upper_upper_s").as_int(),
            node_->get_parameter("color_detection.red_upper_upper_v").as_int()
        );
        
        // 加载蓝色检测参数
        color_params_.blue_lower = cv::Scalar(
            node_->get_parameter("color_detection.blue_lower_h").as_int(),
            node_->get_parameter("color_detection.blue_lower_s").as_int(),
            node_->get_parameter("color_detection.blue_lower_v").as_int()
        );
        color_params_.blue_upper = cv::Scalar(
            node_->get_parameter("color_detection.blue_upper_h").as_int(),
            node_->get_parameter("color_detection.blue_upper_s").as_int(),
            node_->get_parameter("color_detection.blue_upper_v").as_int()
        );
        
        // 加载形态学处理参数
        color_params_.morphology_kernel_size = node_->get_parameter("color_detection.morphology_kernel_size").as_int();
        color_params_.use_adaptive_kernel = node_->get_parameter("color_detection.use_adaptive_kernel").as_bool();
        
        RCLCPP_INFO(node_->get_logger(), "Color detection parameters loaded successfully");
        RCLCPP_INFO(node_->get_logger(), "Red range: [%d,%d,%d] - [%d,%d,%d] and [%d,%d,%d] - [%d,%d,%d]",
                   (int)color_params_.red_lower[0], (int)color_params_.red_lower[1], (int)color_params_.red_lower[2],
                   (int)color_params_.red_upper[0], (int)color_params_.red_upper[1], (int)color_params_.red_upper[2],
                   (int)color_params_.red_upper_lower[0], (int)color_params_.red_upper_lower[1], (int)color_params_.red_upper_lower[2],
                   (int)color_params_.red_upper_upper[0], (int)color_params_.red_upper_upper[1], (int)color_params_.red_upper_upper[2]);
        RCLCPP_INFO(node_->get_logger(), "Blue range: [%d,%d,%d] - [%d,%d,%d]",
                   (int)color_params_.blue_lower[0], (int)color_params_.blue_lower[1], (int)color_params_.blue_lower[2],
                   (int)color_params_.blue_upper[0], (int)color_params_.blue_upper[1], (int)color_params_.blue_upper[2]);
        RCLCPP_INFO(node_->get_logger(), "Morphology kernel size: %d, adaptive: %s",
                   color_params_.morphology_kernel_size, color_params_.use_adaptive_kernel ? "true" : "false");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load color detection parameters: %s", e.what());
        RCLCPP_WARN(node_->get_logger(), "Using default color detection parameters");
    }
}

// 新增：保存调试图像
void FusionStationDetector::saveDebugImage(const std::string& filename, const cv::Mat& image) {
    if (debug_mode_ && !image.empty()) {
        std::string full_path = debug_dir_ + "/" + filename;
        cv::imwrite(full_path, image);
        RCLCPP_INFO(node_->get_logger(), "Saved debug image to: %s", full_path.c_str());
    }
}

// 新增：绘制轮廓
void FusionStationDetector::drawContours(const cv::Mat& original, const std::vector<std::vector<cv::Point>>& contours, const std::string& filename) {
    if (!debug_mode_ || original.empty()) return;
    
    cv::Mat contour_image = original.clone();
    
    // 绘制所有轮廓
    for (size_t i = 0; i < contours.size(); ++i) {
        cv::Scalar color = cv::Scalar(0, 255, 0); // 绿色
        cv::drawContours(contour_image, contours, i, color, 2);
        
        // 添加轮廓编号
        if (!contours[i].empty()) {
            cv::Point center = cv::Point(0, 0);
            for (const auto& pt : contours[i]) {
                center += pt;
            }
            center.x /= contours[i].size();
            center.y /= contours[i].size();
            cv::putText(contour_image, std::to_string(i), center, cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
        }
    }
    
    saveDebugImage(filename, contour_image);
}

// 新增：绘制角点
void FusionStationDetector::drawCorners(const cv::Mat& original, const std::vector<cv::Point2f>& corners, const std::string& filename) {
    if (!debug_mode_ || original.empty() || corners.empty()) return;
    
    cv::Mat corner_image = original.clone();
    
    for (size_t i = 0; i < corners.size(); ++i) {
        cv::Point2f pt = corners[i];
        
        // 绘制角点圆圈
        cv::circle(corner_image, pt, 8, cv::Scalar(0, 0, 255), -1); // 红色实心圆
        
        // 绘制角点编号
        cv::Point text_pt = cv::Point(pt.x + 10, pt.y - 10);
        cv::putText(corner_image, std::to_string(i), text_pt, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    }
    
    saveDebugImage(filename, corner_image);
}

// 新增：绘制拟合的矩形
void FusionStationDetector::drawFittedRectangle(const cv::Mat& original, const std::vector<cv::Point2f>& corners, const std::string& filename) {
    if (!debug_mode_ || original.empty() || corners.size() != 4) return;
    
    cv::Mat rect_image = original.clone();
    
    // 绘制拟合的矩形
    std::vector<cv::Point> rect_points;
    for (const auto& corner : corners) {
        rect_points.push_back(cv::Point(corner.x, corner.y));
    }
    
    // 绘制矩形边框
    for (size_t i = 0; i < rect_points.size(); ++i) {
        cv::Point pt1 = rect_points[i];
        cv::Point pt2 = rect_points[(i + 1) % rect_points.size()];
        cv::line(rect_image, pt1, pt2, cv::Scalar(255, 0, 0), 3); // 蓝色线条
    }
    
    // 绘制角点
    for (size_t i = 0; i < corners.size(); ++i) {
        cv::Point2f pt = corners[i];
        cv::circle(rect_image, pt, 6, cv::Scalar(0, 255, 0), -1); // 绿色实心圆
        cv::Point text_pt = cv::Point(pt.x + 10, pt.y - 10);
        cv::putText(rect_image, std::to_string(i), text_pt, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }
    
    saveDebugImage(filename, rect_image);
}

// 新增：绘制所有调试信息
void FusionStationDetector::drawAllDebugInfo(const cv::Mat& original, const std::vector<std::vector<cv::Point>>& contours, 
                                            const std::vector<cv::Point2f>& corners, const std::string& filename) {
    if (!debug_mode_ || original.empty()) return;
    
    cv::Mat debug_image = original.clone();
    
    // 绘制轮廓
    for (size_t i = 0; i < contours.size(); ++i) {
        cv::Scalar color = cv::Scalar(0, 255, 255); // 黄色
        cv::drawContours(debug_image, contours, i, color, 1);
    }
    
    // 绘制角点
    for (size_t i = 0; i < corners.size(); ++i) {
        cv::Point2f pt = corners[i];
        cv::circle(debug_image, pt, 8, cv::Scalar(0, 0, 255), -1); // 红色实心圆
        cv::Point text_pt = cv::Point(pt.x + 10, pt.y - 10);
        cv::putText(debug_image, std::to_string(i), text_pt, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    }
    
    // 绘制拟合的矩形
    if (corners.size() == 4) {
        std::vector<cv::Point> rect_points;
        for (const auto& corner : corners) {
            rect_points.push_back(cv::Point(corner.x, corner.y));
        }
        
        for (size_t i = 0; i < rect_points.size(); ++i) {
            cv::Point pt1 = rect_points[i];
            cv::Point pt2 = rect_points[(i + 1) % rect_points.size()];
            cv::line(debug_image, pt1, pt2, cv::Scalar(255, 0, 0), 2); // 蓝色线条
        }
    }
    
    // 添加信息文本
    std::string info_text = "Contours: " + std::to_string(contours.size()) + 
                           ", Corners: " + std::to_string(corners.size());
    cv::putText(debug_image, info_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    
    saveDebugImage(filename, debug_image);
}

cv::Point2f FusionStationDetector::findLShapedCorner(const std::vector<cv::Point>& contour) {
    if (contour.size() < 3) {
        return cv::Point2f(0,0);
    }
    
    // 检查轮廓点有效性
    for (const auto& pt : contour) {
        if (pt.x < 0 || pt.y < 0) {
            if (node_) {
                RCLCPP_WARN(node_->get_logger(), "Invalid point in contour: (%d, %d)", pt.x, pt.y);
            }
            // 返回质心作为后备
            cv::Moments m = cv::moments(contour);
            if (m.m00 > 0) {
                return cv::Point2f(m.m10/m.m00, m.m01/m.m00);
            }
            return cv::Point2f(0, 0);
        }
    }
    
    // 1. 计算轮廓凸包
    std::vector<cv::Point> hull;
    cv::convexHull(contour, hull);
    
    if (hull.size() < 3) {
        return cv::Point2f(0,0);
    }
    
    // 2. 寻找L形顶点（90度角点）
    double min_angle_diff = 180.0;
    cv::Point2f best_vertex;
    int best_idx = -1;
    
    for (size_t i = 0; i < hull.size(); i++) {
        cv::Point2f pt0 = hull[(i + hull.size() - 1) % hull.size()];
        cv::Point2f pt1 = hull[i];
        cv::Point2f pt2 = hull[(i + 1) % hull.size()];
        
        cv::Point2f vec1 = pt0 - pt1;
        cv::Point2f vec2 = pt2 - pt1;
        
        double dot = vec1.dot(vec2);
        double norm1 = cv::norm(vec1);
        double norm2 = cv::norm(vec2);
        
        if (norm1 < 1e-5 || norm2 < 1e-5) continue;
        
        double angle_rad = std::acos(dot / (norm1 * norm2));
        double angle_deg = angle_rad * 180.0 / CV_PI;
        
        // 寻找最接近90度的角
        double diff = std::abs(angle_deg - 90.0);
        if (diff < min_angle_diff) {
            min_angle_diff = diff;
            best_vertex = pt1;
            best_idx = i;
        }
    }
    
    // 3. 验证是否为有效L形角点
    if (min_angle_diff < 30.0 && best_idx >= 0) {
        // 检查相邻边长度比（L形特征）
        cv::Point2f prev_pt = hull[(best_idx + hull.size() - 1) % hull.size()];
        cv::Point2f next_pt = hull[(best_idx + 1) % hull.size()];
        
        double len1 = cv::norm(best_vertex - prev_pt);
        double len2 = cv::norm(best_vertex - next_pt);
        double ratio = std::max(len1, len2) / std::min(len1, len2);
        
        // L形两边的长度比应在合理范围内（1:1到1:4）
        if (ratio > 1.5 && ratio < 4.0) {
            return best_vertex;
        }
    }
    
    // 4. 后备方案：返回轮廓质心
    cv::Moments m = cv::moments(contour);
    if (m.m00 > 0) {
        return cv::Point2f(m.m10/m.m00, m.m01/m.m00);
    }
    
    return cv::Point2f(0, 0);
}

std::vector<std::vector<cv::Point>> FusionStationDetector::filterContours(
    const std::vector<std::vector<cv::Point>>& contours) 
{
    if (!node_) {
        throw std::runtime_error("Node pointer is null in filterContours");
    }
    
    std::vector<std::vector<cv::Point>> filtered;
    double min_area = node_->get_parameter("min_contour_area").as_double();
    double max_area = node_->get_parameter("max_contour_area").as_double();
    
    for(size_t i = 0; i < contours.size(); ++i) {
        const auto& contour = contours[i];
        
        // 检查轮廓点有效性
        bool contour_valid = true;
        for (const auto& pt : contour) {
            if (pt.x < 0 || pt.y < 0) {
                contour_valid = false;
                break;
            }
        }
        if (!contour_valid) {
            if (node_) RCLCPP_INFO(node_->get_logger(), "Contour %zu: Invalid contour points detected", i);
            continue;
        }
        
        double area = cv::contourArea(contour);
        
        // 面积筛选 
        if(area < min_area || area > max_area) {
            if (node_) RCLCPP_INFO(node_->get_logger(), "Contour %zu: Area %.1f not in range [%.1f, %.1f]", 
                                   i, area, min_area, max_area);
            continue;
        }
        
        // 长宽比筛选 - L形轮廓通常有较大长宽比
        cv::RotatedRect rect = cv::minAreaRect(contour);
        float width = rect.size.width;
        float height = rect.size.height;
        float aspect_ratio = (width > height) ? width/height : height/width;
        
        if(aspect_ratio < 1.0 || aspect_ratio > 10.0) {
            if (node_) RCLCPP_INFO(node_->get_logger(), "Contour %zu: Aspect ratio %.2f not in range [1.0, 10.0]", 
                                   i, aspect_ratio);
            continue;
        }
        
        // 凸性检测 - L形轮廓凸性较低
        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        double hull_area = cv::contourArea(hull);
        if (hull_area < 1e-5) {
            if (node_) RCLCPP_INFO(node_->get_logger(), "Contour %zu: Hull area too small %.6f", i, hull_area);
            continue;
        }
        
        double solidity = area / hull_area;
        if(solidity > 0.9) {
            if (node_) RCLCPP_INFO(node_->get_logger(), "Contour %zu: Solidity %.3f > 0.9 (too convex)", i, solidity);
            continue;
        }
        
        // 添加轮廓点数量筛选 - 确保轮廓足够复杂
        if(contour.size() < 20) {
            if (node_) RCLCPP_INFO(node_->get_logger(), "Contour %zu: Too few points %zu < 20", i, contour.size());
            continue;
        }
        
        if (node_) RCLCPP_INFO(node_->get_logger(), "Contour %zu: PASSED - Area: %.1f, Aspect: %.2f, Solidity: %.3f, Points: %zu", 
                              i, area, aspect_ratio, solidity, contour.size());
        filtered.push_back(contour);
    }
    
    std::sort(filtered.begin(), filtered.end(), [](const auto& a, const auto& b){
        return cv::contourArea(a) > cv::contourArea(b);
    });
    
    if(filtered.size() > 4) {
        filtered.resize(4);
    }
    
    return filtered;
}

std::vector<cv::Point2f> FusionStationDetector::sortCorners(std::vector<cv::Point2f> corners) {
    if(corners.size() != 4) return corners;
    
    // 按x坐标排序，分为左右两组
    std::sort(corners.begin(), corners.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x < b.x;
    });
    
    std::vector<cv::Point2f> left(corners.begin(), corners.begin() + 2);
    std::vector<cv::Point2f> right(corners.begin() + 2, corners.end());
    
    // 左侧点按y坐标排序（上小下大）
    std::sort(left.begin(), left.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.y < b.y;
    });
    
    // 右侧点按y坐标排序（上小下大）
    std::sort(right.begin(), right.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.y < b.y;
    });
    
    // 返回顺时针顺序：左上 → 右上 → 右下 → 左下
    return {left[0], right[0], right[1], left[1]};
}

std::vector<cv::Point3f> FusionStationDetector::getObjectPoints() const {
    if (!node_) {
        throw std::runtime_error("Node pointer is null in getObjectPoints");
    }
    
    double outer_size = node_->get_parameter("outer_size").as_double(); // 外框尺寸288mm
    double inner_size = node_->get_parameter("inner_size").as_double(); // 内框尺寸240mm
    double light_width = 0.011; // 灯条宽度11mm
    double light_length = 0.050; // 灯条长度50mm
    
    // 计算灯条中心线位置（内框到外框距离的一半）
    double border_gap = (outer_size - inner_size) / 2.0; // 24mm，框的厚度
    double light_center_offset = border_gap / 2.0; // 12mm，灯条中心线到框边缘的距离

    // 计算灯条角点位置（从外框角点向内偏移）
    double half_outer = outer_size / 2.0;
    double light_corner_offset = half_outer - light_center_offset;
    
    return {
        // 灯条角点位置（兑换站中心为原点）
        cv::Point3f(light_corner_offset, -light_corner_offset, 0),  // 右上灯条角点
        cv::Point3f(-light_corner_offset, -light_corner_offset, 0), // 左上灯条角点
        cv::Point3f(-light_corner_offset, light_corner_offset, 0),  // 左下灯条角点
        cv::Point3f(light_corner_offset, light_corner_offset, 0)    // 右下灯条角点
    };
}