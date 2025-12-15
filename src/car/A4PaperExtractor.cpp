#include "A4PaperExtractor.hpp"
#include <iostream>
#include <algorithm>
#include <numeric>

A4PaperExtractor::A4PaperExtractor() {
    // 初始化稳定性历史记录
    stability_history = std::deque<bool>(10, false);
}

A4PaperExtractor::~A4PaperExtractor() {
    // 清理资源
}

void A4PaperExtractor::setDebugMode(bool enable) {
    enableDebug = enable;
}

cv::Mat A4PaperExtractor::adaptiveWhiteBalance(const cv::Mat& image) {
    /** 自适应白平衡，减少色偏 */
    cv::Mat lab_image;
    cv::cvtColor(image, lab_image, cv::COLOR_BGR2Lab);
    
    std::vector<cv::Mat> lab_channels;
    cv::split(lab_image, lab_channels);
    
    cv::Scalar avg_a = cv::mean(lab_channels[1]);
    cv::Scalar avg_b = cv::mean(lab_channels[2]);
    
    lab_channels[1] = lab_channels[1] - ((avg_a[0] - 128) * 0.7);
    lab_channels[2] = lab_channels[2] - ((avg_b[0] - 128) * 0.7);
    
    cv::Mat result;
    cv::merge(lab_channels, result);
    cv::cvtColor(result, result, cv::COLOR_Lab2BGR);
    
    return result;
}

cv::Mat A4PaperExtractor::detectRedRegionsRobust(const cv::Mat& image) {
    /** 鲁棒的红色区域检测 */
    cv::Mat balanced = adaptiveWhiteBalance(image);
    cv::Mat blurred;
    cv::GaussianBlur(balanced, blurred, cv::Size(5, 5), 0);
    
    cv::Mat hsv;
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
    
    // 定义红色的HSV范围（两个区间）
    cv::Scalar lower_red1(0, 60, 60);
    cv::Scalar upper_red1(15, 255, 255);
    cv::Scalar lower_red2(160, 60, 60);
    cv::Scalar upper_red2(179, 255, 255);
    
    cv::Mat mask1, mask2, red_mask;
    cv::inRange(hsv, lower_red1, upper_red1, mask1);
    cv::inRange(hsv, lower_red2, upper_red2, mask2);
    cv::bitwise_or(mask1, mask2, red_mask);
    
    return red_mask;
}

cv::Mat A4PaperExtractor::completeIncompleteBorder(const cv::Mat& mask, double min_contour_length_ratio) {
    /** 处理不完整边框 */
    int height = mask.rows;
    int width = mask.cols;
    double min_contour_length = std::min(height, width) * min_contour_length_ratio;
    
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    if (contours.empty()) {
        return mask.clone();
    }
    
    // 找到最大轮廓
    auto main_contour = *std::max_element(contours.begin(), contours.end(),
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return cv::contourArea(a) < cv::contourArea(b);
        });
    
    double perimeter = cv::arcLength(main_contour, true);
    double epsilon = 0.02 * perimeter;
    std::vector<cv::Point> approx;
    cv::approxPolyDP(main_contour, approx, epsilon, true);
    
    if (approx.size() < 4) {
        std::vector<cv::Point> hull;
        cv::convexHull(main_contour, hull);
        cv::Mat completed_mask = cv::Mat::zeros(mask.size(), mask.type());
        std::vector<std::vector<cv::Point>> hull_vec = {hull};
        cv::fillPoly(completed_mask, hull_vec, cv::Scalar(255));
        
        cv::Mat combined_mask;
        cv::bitwise_or(mask, completed_mask, combined_mask);
        return combined_mask;
    }
    
    return mask.clone();
}

cv::Mat A4PaperExtractor::refineRedMaskAdvanced(const cv::Mat& mask, const cv::Mat& original_image) {
    /** 高级红色掩膜优化 */
    int height = mask.rows;
    int width = mask.cols;
    double min_area = height * width * 0.01;
    
    cv::Mat completed_mask = completeIncompleteBorder(mask);
    
    int kernel_size = std::max(3, static_cast<int>(std::min(height, width) * 0.01));
    cv::Mat kernel_open = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
    cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size * 4, kernel_size * 4));
    
    cv::Mat processed;
    cv::morphologyEx(completed_mask, processed, cv::MORPH_OPEN, kernel_open);
    cv::morphologyEx(processed, processed, cv::MORPH_CLOSE, kernel_close);
    
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(processed, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    cv::Mat final_mask = cv::Mat::zeros(processed.size(), processed.type());
    
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > min_area) {
            std::vector<cv::Point> hull;
            cv::convexHull(contour, hull);
            std::vector<std::vector<cv::Point>> hull_vec = {hull};
            cv::fillPoly(final_mask, hull_vec, cv::Scalar(255));
        }
    }
    
    return final_mask;
}

bool A4PaperExtractor::detectA4Quadrilateral(const cv::Mat& mask, const cv::Mat& original_image, 
                                            std::vector<cv::Point2f>& quad, double aspect_tolerance) {
    /** 灵活的A4纸四边形检测 */
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    if (contours.empty()) {
        return false;
    }
    
    // 按面积排序
    std::sort(contours.begin(), contours.end(), 
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return cv::contourArea(a) > cv::contourArea(b);
        });
    
    std::vector<cv::Point2f> best_quad;
    double best_score = 0;
    double target_aspect = 297.0 / 210.0;
    
    for (const auto& contour : contours) {
        if (cv::contourArea(contour) < 5000) {
            continue;
        }
        
        double peri = cv::arcLength(contour, true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, 0.02 * peri, true);
        
        if (approx.size() >= 3 && approx.size() <= 6) {
            std::vector<cv::Point> processed_approx = approx;
            
            if (approx.size() != 4) {
                cv::RotatedRect rect = cv::minAreaRect(approx);
                cv::Point2f vertices[4];
                rect.points(vertices);
                processed_approx.clear();
                for (int i = 0; i < 4; i++) {
                    processed_approx.push_back(cv::Point(cvRound(vertices[i].x), cvRound(vertices[i].y)));
                }
            }
            
            if (!cv::isContourConvex(processed_approx)) {
                std::vector<cv::Point> hull;
                cv::convexHull(processed_approx, hull);
                processed_approx = hull;
            }
            
            if (processed_approx.size() >= 4) {
                std::vector<cv::Point> quad_points = processed_approx;
                if (quad_points.size() > 4) {
                    std::vector<cv::Point> hull;
                    cv::convexHull(quad_points, hull);
                    if (hull.size() >= 4) {
                        quad_points = std::vector<cv::Point>(hull.begin(), hull.begin() + 4);
                    } else {
                        continue;
                    }
                }
                
                // 计算最小外接矩形
                cv::RotatedRect rect = cv::minAreaRect(quad_points);
                cv::Point2f vertices[4];
                rect.points(vertices);
                
                double width = cv::norm(vertices[0] - vertices[1]);
                double height = cv::norm(vertices[1] - vertices[2]);
                double aspect_ratio = (width > 0 && height > 0) ? 
                    std::max(width, height) / std::min(width, height) : 0;
                double aspect_diff = std::abs(aspect_ratio - target_aspect);
                
                if (aspect_diff <= aspect_tolerance) {
                    double area = cv::contourArea(quad_points);
                    std::vector<cv::Point> hull;
                    cv::convexHull(quad_points, hull);
                    double hull_area = cv::contourArea(hull);
                    double convexity = (hull_area > 0) ? area / hull_area : 0;
                    double score = area * (1 - aspect_diff) * convexity;
                    
                    if (score > best_score) {
                        best_score = score;
                        best_quad.clear();
                        for (const auto& p : quad_points) {
                            best_quad.push_back(cv::Point2f(p.x, p.y));
                        }
                    }
                }
            }
        }
    }
    
    if (!best_quad.empty()) {
        quad = best_quad;
        return true;
    }
    
    return false;
}

std::vector<cv::Point2f> A4PaperExtractor::improvedOrderPoints(const std::vector<cv::Point2f>& pts) {
    /** 改进的点排序方法 */
    if (pts.size() != 4) {
        return std::vector<cv::Point2f>();
    }
    
    std::vector<cv::Point2f> points = pts;
    std::vector<float> sum_pts, diff_pts;
    
    for (const auto& p : points) {
        sum_pts.push_back(p.x + p.y);
        diff_pts.push_back(p.y - p.x); // 注意：这里使用y-x来匹配Python的diff逻辑
    }
    
    cv::Point2f top_left = points[std::min_element(sum_pts.begin(), sum_pts.end()) - sum_pts.begin()];
    cv::Point2f bottom_right = points[std::max_element(sum_pts.begin(), sum_pts.end()) - sum_pts.begin()];
    cv::Point2f top_right = points[std::min_element(diff_pts.begin(), diff_pts.end()) - diff_pts.begin()];
    cv::Point2f bottom_left = points[std::max_element(diff_pts.begin(), diff_pts.end()) - diff_pts.begin()];
    
    std::vector<cv::Point2f> result = {top_left, top_right, bottom_right, bottom_left};
    return result;
}

cv::Mat A4PaperExtractor::perspectiveTransform(const cv::Mat& image, const std::vector<cv::Point2f>& quad) {
    /** 透视变换 */
    if (quad.size() != 4) {
        return cv::Mat();
    }
    
    try {
        std::vector<cv::Point2f> src_pts = improvedOrderPoints(quad);
        int width = 600;
        int height = static_cast<int>(width * TARGET_ASPECT);
        
        std::vector<cv::Point2f> dst_pts = {
            cv::Point2f(0, 0),
            cv::Point2f(width-1, 0),
            cv::Point2f(width-1, height-1),
            cv::Point2f(0, height-1)
        };
        
        cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
        cv::Mat warped;
        cv::warpPerspective(image, warped, M, cv::Size(width, height));
        
        if (apply_flip_correction) {
            cv::flip(warped, warped, 1);
        }
        
        return warped;
        
    } catch (const std::exception& e) {
        std::cout << "透视变换错误: " << e.what() << std::endl;
        return cv::Mat();
    }
}

float A4PaperExtractor::calculateConfidence(const cv::Mat& original, const cv::Mat& warped, 
                                           const std::vector<cv::Point2f>& quad) {
    /** 分析内部内容提取效果 */
    if (warped.empty()) {
        return 0.0f;
    }
    
    cv::Mat gray_warped;
    cv::cvtColor(warped, gray_warped, cv::COLOR_BGR2GRAY);
    cv::Scalar mean_w, stddev_w;
    cv::meanStdDev(gray_warped, mean_w, stddev_w);
    double content_variance = stddev_w[0] * stddev_w[0];
    
    cv::Mat mask = cv::Mat::zeros(original.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> quad_vec;
    std::vector<cv::Point> int_quad;
    for (const auto& p : quad) {
        int_quad.push_back(cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)));
    }
    quad_vec.push_back(int_quad);
    cv::fillPoly(mask, quad_vec, cv::Scalar(255));
    
    cv::Mat original_roi;
    original.copyTo(original_roi, mask);
    
    cv::Mat gray_original;
    cv::cvtColor(original_roi, gray_original, cv::COLOR_BGR2GRAY);
    
    cv::Mat masked_gray;
    gray_original.copyTo(masked_gray, mask);
    
    cv::Scalar mean_original, stddev_original;
    cv::meanStdDev(masked_gray, mean_original, stddev_original, mask);
    double original_variance = stddev_original[0] * stddev_original[0];
    
    double content_preservation = 0.0;
    if (original_variance > 0) {
        content_preservation = std::min(content_variance / original_variance, 1.0);
    }
    
    return static_cast<float>(content_preservation);
}

cv::Mat A4PaperExtractor::extract(const cv::Mat& frame) {
    /**
     * 处理单帧图像，返回提取的A4纸
     */
    if (frame.empty()) {
        return cv::Mat();
    }
    
    // 显示原图
    if (enableDebug) {
        cv::imshow("1. Original", frame);
    }
    
    // 1. 红色区域检测
    cv::Mat red_mask = detectRedRegionsRobust(frame);
    
    // 显示红色掩膜
    if (enableDebug) {
        cv::imshow("2. Red Mask", red_mask);
    }
    
    // 2. 掩膜优化
    cv::Mat refined_mask = refineRedMaskAdvanced(red_mask, frame);
    
    // 显示优化后的掩膜
    if (enableDebug) {
        cv::imshow("3. Refined Mask", refined_mask);
    }
    
    // 3. 检测A4四边形
    std::vector<cv::Point2f> quad;
    bool detect_ok = detectA4Quadrilateral(refined_mask, frame, quad);
    
    if (!detect_ok) {
        // 显示检测失败
        if (enableDebug) {
            cv::Mat result_display = frame.clone();
            cv::putText(result_display, "No A4 Paper Detected", cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            cv::imshow("4. Detection Result", result_display);
            cv::waitKey(1);
        }
        return cv::Mat();
    }
    
    // 显示检测结果
    if (enableDebug) {
        cv::Mat result_display = frame.clone();
        // 绘制四边形
        for (int i = 0; i < 4; i++) {
            cv::line(result_display, quad[i], quad[(i + 1) % 4], cv::Scalar(0, 255, 0), 3);
        }
        // 绘制顶点
        for (int i = 0; i < 4; i++) {
            cv::circle(result_display, quad[i], 8, cv::Scalar(0, 0, 255), -1);
            cv::putText(result_display, std::to_string(i), quad[i], 
                       cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        }
        cv::imshow("4. Detection Result", result_display);
    }
    
    // 4. 透视变换提取A4纸
    cv::Mat extracted = perspectiveTransform(frame, quad);
    if (extracted.empty()) {
        return cv::Mat();
    }
    
    // 5. 置信度筛选
    float confidence = calculateConfidence(frame, extracted, quad);
    if (confidence < CONFIDENCE_THRESHOLD) {
        return cv::Mat();
    }
    
    // 显示提取结果
    if (enableDebug) {
        cv::Mat extracted_display = extracted.clone();
        std::string conf_text = "Confidence: " + std::to_string(confidence);
        cv::putText(extracted_display, conf_text, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        cv::imshow("5. Extracted A4", extracted_display);
        
        // 等待按键
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) { // q或ESC退出
            return cv::Mat();
        } 
    }
    
    // 6. 稳定性记录
    stability_history.push_back(confidence > 0.3f);
    if (stability_history.size() > 10) {
        stability_history.pop_front();
    }
    
    return extracted;
}