#ifndef __CONE_DETECTOR_HPP
#define __CONE_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>
#include "config.hpp"
#include "image_Q.hpp"

namespace ConeDetector
{
    // ============ 配置常数 ============
    constexpr int DEFAULT_MAX_CONES = 6;                   // 默认最多识别锥桶数
    constexpr int DEFAULT_MAX_RED_CONES = 4;               // 默认最多识别红色锥桶数
    constexpr int DEFAULT_MORPH_KERNEL = 5;
    constexpr double DEFAULT_MIN_AREA = 150.0;
    constexpr double DEFAULT_MAX_AREA = 1000.0;
    constexpr double DEFAULT_RATIO_MIN = 0.4;
    constexpr double DEFAULT_RATIO_MAX = 4.0;
    constexpr double DEFAULT_TRACKING_DIST = 50.0;
    constexpr int DEFAULT_MAX_DISAPPEARED = 10;

    // ============ 数据结构 ============
    struct ConeParams {
        cv::Scalar hsv_low, hsv_high;
        int morph_kernel_size = DEFAULT_MORPH_KERNEL;
        double min_area = DEFAULT_MIN_AREA;
        double max_area = DEFAULT_MAX_AREA;
        double area_ratio_min = DEFAULT_RATIO_MIN;
        double area_ratio_max = DEFAULT_RATIO_MAX;
        double tracking_distance_threshold = DEFAULT_TRACKING_DIST;
        int max_disappeared_frames = DEFAULT_MAX_DISAPPEARED;
        int max_cones = DEFAULT_MAX_CONES;  // 添加最大锥桶数配置
    };

    struct ConeObject {
        int id;
        cv::Rect bounding_box;
        cv::Point center;
        double area;
        int disappeared_frames;
        bool is_visible;
    };

    // ============ 全局变量 ============
    inline ConeParams detection_params;
    inline ConeParams red_detection_params;
    inline std::vector<ConeObject> detected_cones;
    inline std::vector<ConeObject> detected_red_cones;
    inline std::map<int, ConeObject> tracked_cones;
    inline std::map<int, ConeObject> tracked_red_cones;
    inline int next_cone_id = 0;
    inline int next_red_cone_id = 0;
    inline std::vector<cv::Point> line_points;
    inline std::vector<cv::Point> red_line_points;
    inline std::vector<cv::Point> real_line_points;      // 基于真实检测锥桶的路径
    inline std::vector<cv::Point> real_red_line_points;  // 基于真实检测红色锥桶的路径
    inline std::vector<ConeObject> real_cones;           // 真实检测的锥桶（不含虚拟点）
    inline std::vector<ConeObject> real_red_cones;       // 真实检测的红色锥桶（不含虚拟点）
    inline int frame_size_width = 0;
    inline int frame_size_height = 0;
    inline int error_offset = 0;
    inline int error_scale = 30;

    // ============ 核心函数 ============
    inline void initConeDetector(const cv::Scalar& hsv_low, const cv::Scalar& hsv_high,
                                 double min_area = DEFAULT_MIN_AREA, double max_area = DEFAULT_MAX_AREA,
                                 int max_cones = DEFAULT_MAX_CONES) {
        detection_params.hsv_low = hsv_low;
        detection_params.hsv_high = hsv_high;
        detection_params.min_area = min_area;
        detection_params.max_area = max_area;
        detection_params.max_cones = max_cones;  // 设置最大锥桶数
        next_cone_id = 0;
        try
        {
            auto config = Config::get_config();
            error_offset = config["vision"]["cone_detection"]["error_offset"].get<int>();
            error_scale = config["vision"]["cone_detection"]["error_scale"].get<int>();
        } catch (const std::exception& e)
        {
            std::cout << e.what() << std::endl;
        }

        tracked_cones.clear();
    }

    /**
     * @brief 初始化红色锥桶检测器
     */
    inline void initRedConeDetector(const cv::Scalar& hsv_low, const cv::Scalar& hsv_high,
                                     double min_area = DEFAULT_MIN_AREA, double max_area = DEFAULT_MAX_AREA,
                                     int max_cones = DEFAULT_MAX_RED_CONES) {
        red_detection_params.hsv_low = hsv_low;
        red_detection_params.hsv_high = hsv_high;
        red_detection_params.min_area = min_area;
        red_detection_params.max_area = max_area;
        red_detection_params.max_cones = max_cones;  // 设置最大红色锥桶数
        next_red_cone_id = 0;
        tracked_red_cones.clear();
    }

    inline double calculateDistance(const cv::Point& p1, const cv::Point& p2) {
        double dx = p1.x - p2.x, dy = p1.y - p2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    inline int getNextAvailableId() {
        return (next_cone_id >= detection_params.max_cones) ? -1 : next_cone_id++;
    }

    inline int getNextAvailableRedId() {
        return (next_red_cone_id >= red_detection_params.max_cones) ? -1 : next_red_cone_id++;
    }



    inline std::vector<ConeObject> matchDetectionsToTracks(const std::vector<ConeObject>& new_detections) {
        std::vector<ConeObject> matched_cones;
        std::vector<bool> used(new_detections.size(), false);

        // 清理离线锥桶
        std::vector<int> to_remove;
        for (auto& [id, cone] : tracked_cones) {
            if (cone.disappeared_frames > detection_params.max_disappeared_frames) {
                to_remove.push_back(id);
            }
        }
        for (int id : to_remove) tracked_cones.erase(id);

        // 匹配已有追踪与新检测
        for (auto& [id, tracked] : tracked_cones) {
            tracked.is_visible = false;
            tracked.disappeared_frames++;

            double min_dist = detection_params.tracking_distance_threshold;
            int best_idx = -1;

            for (size_t i = 0; i < new_detections.size(); ++i) {
                if (used[i]) continue;
                double dist = calculateDistance(tracked.center, new_detections[i].center);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_idx = static_cast<int>(i);
                }
            }

            if (best_idx >= 0) {
                auto upd = new_detections[best_idx];
                upd.id = id;
                upd.disappeared_frames = 0;
                upd.is_visible = true;
                tracked_cones[id] = upd;
                matched_cones.push_back(upd);
                used[best_idx] = true;
            }
        }

        // 为新检测分配 ID
        for (size_t i = 0; i < new_detections.size(); i++) {
            if (used[i]) continue;
            int new_id = getNextAvailableId();
            if (new_id == -1) continue;

            auto cone = new_detections[i];
            cone.id = new_id;
            cone.disappeared_frames = 0;
            cone.is_visible = true;
            tracked_cones[new_id] = cone;
            matched_cones.push_back(cone);
        }

        return matched_cones;
    }

    inline std::vector<ConeObject> matchRedDetectionsToTracks(const std::vector<ConeObject>& new_detections) {
        std::vector<ConeObject> matched_cones;
        std::vector<bool> used(new_detections.size(), false);

        // 清理离线红色锥桶
        std::vector<int> to_remove;
        for (auto& [id, cone] : tracked_red_cones) {
            if (cone.disappeared_frames > red_detection_params.max_disappeared_frames) {
                to_remove.push_back(id);
            }
        }
        for (int id : to_remove) tracked_red_cones.erase(id);

        // 匹配已有追踪与新检测
        for (auto& [id, tracked] : tracked_red_cones) {
            tracked.is_visible = false;
            tracked.disappeared_frames++;

            double min_dist = red_detection_params.tracking_distance_threshold;
            int best_idx = -1;

            for (size_t i = 0; i < new_detections.size(); ++i) {
                if (used[i]) continue;
                double dist = calculateDistance(tracked.center, new_detections[i].center);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_idx = static_cast<int>(i);
                }
            }

            if (best_idx >= 0) {
                auto upd = new_detections[best_idx];
                upd.id = id;
                upd.disappeared_frames = 0;
                upd.is_visible = true;
                tracked_red_cones[id] = upd;
                matched_cones.push_back(upd);
                used[best_idx] = true;
            }
        }

        // 为新检测分配 ID
        for (size_t i = 0; i < new_detections.size(); ++i) {
            if (used[i]) continue;
            int new_id = getNextAvailableRedId();
            if (new_id == -1) continue;

            auto cone = new_detections[i];
            cone.id = new_id;
            cone.disappeared_frames = 0;
            cone.is_visible = true;
            tracked_red_cones[new_id] = cone;
            matched_cones.push_back(cone);
        }

        return matched_cones;
    }

    /**
     * @brief 计算真实锥桶间的路径点（用于误差计算）
     */
    inline void computeRealLinePath() {
        real_line_points.clear();

        if (real_cones.size() < 2) {
            return;  // 少于2个锥桶无法补线
        }

        // 按 X 坐标排序锥桶（从左到右）
        std::vector<ConeObject> sorted_cones = real_cones;
        std::sort(sorted_cones.begin(), sorted_cones.end(),
                  [](const ConeObject& a, const ConeObject& b) {
                      return a.center.x < b.center.x;
                  });

        // 连接相邻锥桶的中心点
        for (size_t i = 0; i < sorted_cones.size(); ++i) {
            real_line_points.push_back(sorted_cones[i].center);

            // 在相邻锥桶间进行线性插值
            if (i < sorted_cones.size() - 1) {
                cv::Point p1 = sorted_cones[i].center;
                cv::Point p2 = sorted_cones[i + 1].center;

                // 计算两点间的距离
                double dx = p2.x - p1.x;
                double dy = p2.y - p1.y;
                double dist = std::sqrt(dx * dx + dy * dy);

                // 每 10 像素插入一个点
                int steps = static_cast<int>(dist / 10.0);
                for (int j = 1; j < steps; ++j) {
                    double t = static_cast<double>(j) / steps;
                    cv::Point interp(p1.x + dx * t, p1.y + dy * t);
                    real_line_points.push_back(interp);
                }
            }
        }
    }

    /**
     * @brief 计算相邻锥桶间的补线路径点（包含虚拟点，用于显示）
     */
    inline void computeLinePath() {
        line_points.clear();

        if (detected_cones.size() < 2) {
            return;  // 少于2个锥桶无法补线
        }

        // 按 X 坐标排序锥桶（从左到右）
        std::vector<ConeObject> sorted_cones = detected_cones;
        std::sort(sorted_cones.begin(), sorted_cones.end(),
                  [](const ConeObject& a, const ConeObject& b) {
                      return a.center.x < b.center.x;
                  });

        // 连接相邻锥桶的中心点
        for (size_t i = 0; i < sorted_cones.size(); ++i) {
            line_points.push_back(sorted_cones[i].center);

            // 在相邻锥桶间进行线性插值
            if (i < sorted_cones.size() - 1) {
                cv::Point p1 = sorted_cones[i].center;
                cv::Point p2 = sorted_cones[i + 1].center;

                // 计算两点间的距离
                double dx = p2.x - p1.x;
                double dy = p2.y - p1.y;
                double dist = std::sqrt(dx * dx + dy * dy);

                // 每 10 像素插入一个点
                int steps = static_cast<int>(dist / 10.0);
                for (int j = 1; j < steps; ++j) {
                    double t = static_cast<double>(j) / steps;
                    cv::Point interp(p1.x + dx * t, p1.y + dy * t);
                    line_points.push_back(interp);
                }
            }
        }
    }

    /**
     * @brief 计算真实红色锥桶间的路径点（用于误差计算）
     */
    inline void computeRealRedLinePath() {
        real_red_line_points.clear();

        if (real_red_cones.size() < 2) {
            return;
        }

        std::vector<ConeObject> sorted_cones = real_red_cones;
        std::sort(sorted_cones.begin(), sorted_cones.end(),
                  [](const ConeObject& a, const ConeObject& b) {
                      return a.center.x < b.center.x;
                  });

        for (size_t i = 0; i < sorted_cones.size(); ++i) {
            real_red_line_points.push_back(sorted_cones[i].center);

            if (i < sorted_cones.size() - 1) {
                cv::Point p1 = sorted_cones[i].center;
                cv::Point p2 = sorted_cones[i + 1].center;

                double dx = p2.x - p1.x;
                double dy = p2.y - p1.y;
                double dist = std::sqrt(dx * dx + dy * dy);

                int steps = static_cast<int>(dist / 10.0);
                for (int j = 1; j < steps; ++j) {
                    double t = static_cast<double>(j) / steps;
                    cv::Point interp(p1.x + dx * t, p1.y + dy * t);
                    real_red_line_points.push_back(interp);
                }
            }
        }
    }

    /**
     * @brief 计算红色锥桶间的补线路径点（包含虚拟点，用于显示）
     */
    inline void computeRedLinePath() {
        red_line_points.clear();

        if (detected_red_cones.size() < 2) {
            return;
        }

        std::vector<ConeObject> sorted_cones = detected_red_cones;
        std::sort(sorted_cones.begin(), sorted_cones.end(),
                  [](const ConeObject& a, const ConeObject& b) {
                      return a.center.x < b.center.x;
                  });

        for (size_t i = 0; i < sorted_cones.size(); ++i) {
            red_line_points.push_back(sorted_cones[i].center);

            if (i < sorted_cones.size() - 1) {
                cv::Point p1 = sorted_cones[i].center;
                cv::Point p2 = sorted_cones[i + 1].center;

                double dx = p2.x - p1.x;
                double dy = p2.y - p1.y;
                double dist = std::sqrt(dx * dx + dy * dy);

                int steps = static_cast<int>(dist / 10.0);
                for (int j = 1; j < steps; ++j) {
                    double t = static_cast<double>(j) / steps;
                    cv::Point interp(p1.x + dx * t, p1.y + dy * t);
                    red_line_points.push_back(interp);
                }
            }
        }
    }


    /**
     * @brief 根据已有锥桶推算虚拟点
     * @param cones 锥桶列表
     * @param extrapolate_head 是否在头部推算点
     * @param extrapolate_tail 是否在尾部推算点
     */
    inline void extrapolateConePoints(std::vector<ConeObject>& cones, bool extrapolate_head = false, bool extrapolate_tail = true) {
        if (cones.size() < 2) {
            return;  // 需要至少2个点才能推算
        }

        // 尾部推算 (基于最后两个点)
        if (extrapolate_tail) {
            auto& p1 = cones[cones.size() - 2].center;
            auto& p2 = cones[cones.size() - 1].center;

            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;

            cv::Point new_point;
            if (std::abs(dx) > 0.1) {
                new_point.x = p2.x + static_cast<int>(dx);
                new_point.y = p2.y + static_cast<int>(dy);
            } else {
                // 垂直线的情况
                new_point.x = p2.x;
                new_point.y = p2.y + static_cast<int>(dy);
            }

            auto new_cone = ConeObject {
                .id = -1,
                .bounding_box = cv::Rect(new_point.x - 10, new_point.y - 10, 20, 20),
                .center = new_point,
                .area = 500.0,
                .disappeared_frames = 0,
                .is_visible = true
            };

            cones.push_back(new_cone);
        }

        // 头部推算 (基于前两个点)
        if (extrapolate_head) {
            auto& p1 = cones[0].center;
            auto& p2 = cones[1].center;

            double dx = p1.x - p2.x;  // 反向计算
            double dy = p1.y - p2.y;

            cv::Point new_point;
            if (std::abs(dx) > 0.1) {
                new_point.x = p1.x + static_cast<int>(dx);
                new_point.y = p1.y + static_cast<int>(dy);
            } else {
                // 垂直线的情况
                new_point.x = p1.x;
                new_point.y = p1.y + static_cast<int>(dy);
            }

            auto new_cone = ConeObject {
                .id = -1,
                .bounding_box = cv::Rect(new_point.x - 10, new_point.y - 10, 20, 20),
                .center = new_point,
                .area = 500.0,
                .disappeared_frames = 0,
                .is_visible = true
            };

            cones.insert(cones.begin(), new_cone);
        }
    }

    inline std::vector<ConeObject> detectCones(const cv::Mat& frame) {
        detected_cones.clear();
        real_cones.clear();  // 清空真实锥桶列表
        frame_size_height = frame.rows;
        frame_size_width = frame.cols;

        // HSV 检测
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, detection_params.hsv_low, detection_params.hsv_high, mask);

        // 形态学操作
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                    cv::Size(detection_params.morph_kernel_size,
                                                           detection_params.morph_kernel_size));
        cv::Mat opening, dilated;
        cv::morphologyEx(mask, opening, cv::MORPH_OPEN, kernel);
        cv::dilate(opening, dilated, kernel, cv::Point(-1, -1), 2);

        // 轮廓检测
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dilated.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<ConeObject> raw_detections;
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < detection_params.min_area || area > detection_params.max_area) continue;

            cv::Rect bbox = cv::boundingRect(contour);
            double ratio = static_cast<double>(bbox.width) / std::max(1, bbox.height);
            if (ratio < detection_params.area_ratio_min || ratio > detection_params.area_ratio_max) continue;

            ConeObject cone{-1, bbox, cv::Point(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2),
                           area, 0, false};
            raw_detections.push_back(cone);
        }

        detected_cones = matchDetectionsToTracks(raw_detections);

        // 过滤有效锥桶
        std::vector<ConeObject> valid;
        for (const auto& cone : detected_cones) {
            if (cone.id >= 0 && cone.id < detection_params.max_cones) {
                valid.push_back(cone);
            }
        }
        detected_cones = valid;

        // 保存真实检测的锥桶（用于误差计算）
        real_cones = detected_cones;

        // 基于真实锥桶计算路径（用于误差计算）
        computeRealLinePath();

        // 进行虚拟补点（用于显示和预测）
        extrapolateConePoints(detected_cones, true, false);

        // 计算包含虚拟点的完整路径（用于显示）
        computeLinePath();

        return detected_cones;
    }

    /**
     * @brief 检测红色锥桶
     */
    inline std::vector<ConeObject> detectRedCones(const cv::Mat& frame) {
        detected_red_cones.clear();
        real_red_cones.clear();  // 清空真实红色锥桶列表

        // HSV 检测 - 红色需要两个范围
        cv::Mat hsv, mask1, mask2, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // 第一个红色范围: 0-10 (低色调红色)
        cv::inRange(hsv, red_detection_params.hsv_low, red_detection_params.hsv_high, mask1);

        // 第二个红色范围: 170-180 (高色调红色)
        cv::Scalar hsv_low2(170, red_detection_params.hsv_low[1], red_detection_params.hsv_low[2]);
        cv::Scalar hsv_high2(180, red_detection_params.hsv_high[1], red_detection_params.hsv_high[2]);
        cv::inRange(hsv, hsv_low2, hsv_high2, mask2);

        // 合并两个掩码
        cv::bitwise_or(mask1, mask2, mask);
        // 形态学操作
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                    cv::Size(red_detection_params.morph_kernel_size,
                                                           red_detection_params.morph_kernel_size));
        cv::Mat opening, dilated;
        cv::morphologyEx(mask, opening, cv::MORPH_OPEN, kernel);
        cv::dilate(opening, dilated, kernel, cv::Point(-1, -1), 2);

        // 轮廓检测
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dilated.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<ConeObject> raw_detections;
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < red_detection_params.min_area || area > red_detection_params.max_area) continue;

            cv::Rect bbox = cv::boundingRect(contour);
            double ratio = static_cast<double>(bbox.width) / std::max(1, bbox.height);
            if (ratio < red_detection_params.area_ratio_min || ratio > red_detection_params.area_ratio_max) continue;

            ConeObject cone{-1, bbox, cv::Point(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2),
                           area, 0, false};
            raw_detections.push_back(cone);
        }

        detected_red_cones = matchRedDetectionsToTracks(raw_detections);

        // 过滤有效红色锥桶
        std::vector<ConeObject> valid;
        for (const auto& cone : detected_red_cones) {
            if (cone.id >= 0 && cone.id < red_detection_params.max_cones) {
                valid.push_back(cone);
            }
        }
        detected_red_cones = valid;

        // 保存真实检测的红色锥桶（用于误差计算）
        real_red_cones = detected_red_cones;

        // 基于真实红色锥桶计算路径（用于误差计算）
        computeRealRedLinePath();

        // 进行虚拟补点（用于显示和预测）
        extrapolateConePoints(detected_red_cones, true, false);

        // 计算包含虚拟点的完整路径（用于显示）
        computeRedLinePath();

        return detected_red_cones;
    }

    /**
     * @brief 获取补线后的路径点
     * @return 路径点列表
     */
    inline const std::vector<cv::Point>& getLinePath() {
        return line_points;
    }

    /**
     * @brief 获取红色锥桶补线路径
     */
    inline const std::vector<cv::Point>& getRedLinePath() {
        return red_line_points;
    }

    /**
     * @brief 获取真实锥桶路径点（用于误差计算）
     * @return 真实路径点列表
     */
    inline const std::vector<cv::Point>& getRealLinePath() {
        return real_line_points;
    }

    /**
     * @brief 获取真实红色锥桶路径点（用于误差计算）
     */
    inline const std::vector<cv::Point>& getRealRedLinePath() {
        return real_red_line_points;
    }

    /**
     * @brief 获取真实检测的锥桶列表
     */
    inline const std::vector<ConeObject>& getRealCones() {
        return real_cones;
    }

    /**
     * @brief 获取真实检测的红色锥桶列表
     */
    inline const std::vector<ConeObject>& getRealRedCones() {
        return real_red_cones;
    }

    inline void drawDetectedCones(cv::Mat& frame, bool draw_info = true) {
        const cv::Scalar GREEN(0, 255, 0), YELLOW(0, 255, 255), RED(0, 0, 255), CYAN(255, 255, 0);

        // 绘制补线路径
        if (line_points.size() > 1) {
            for (size_t i = 0; i < line_points.size() - 1; ++i) {
                cv::line(frame, line_points[i], line_points[i + 1], CYAN, 2);
            }
        }

        for (const auto& cone : detected_cones) {
            // 绘制边框
            cv::rectangle(frame, cone.bounding_box, GREEN, 2);
            cv::rectangle(frame, cv::Point(cone.bounding_box.x - 2, cone.bounding_box.y - 2),
                         cv::Point(cone.bounding_box.x + cone.bounding_box.width + 2,
                                  cone.bounding_box.y + cone.bounding_box.height + 2), YELLOW, 1);

            // 绘制中心
            cv::circle(frame, cone.center, 8, RED, -1);
            cv::circle(frame, cone.center, 8, YELLOW, 2);

            // 绘制十字
            cv::line(frame, cv::Point(cone.center.x - 15, cone.center.y),
                     cv::Point(cone.center.x + 15, cone.center.y), CYAN, 1);
            cv::line(frame, cv::Point(cone.center.x, cone.center.y - 15),
                     cv::Point(cone.center.x, cone.center.y + 15), CYAN, 1);

            if (draw_info) {
                // 绘制 ID
                std::string id_text = "ID #" + std::to_string(cone.id);
                cv::putText(frame, id_text, cv::Point(cone.bounding_box.x, cone.bounding_box.y - 5),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, YELLOW, 2);

                // 绘制位置和面积
                std::string info = "(" + std::to_string(cone.center.x) + "," + std::to_string(cone.center.y) + ")";
                cv::putText(frame, info, cv::Point(cone.center.x + 15, cone.center.y - 5),
                           cv::FONT_HERSHEY_SIMPLEX, 0.4, CYAN, 1);
                cv::putText(frame, "A:" + std::to_string(static_cast<int>(cone.area)),
                           cv::Point(cone.bounding_box.x, cone.bounding_box.y + cone.bounding_box.height + 15),
                           cv::FONT_HERSHEY_SIMPLEX, 0.4, CYAN, 1);
            }
        }

        // 绘制统计信息
        std::string stat = "V:" + std::to_string(detected_cones.size()) + " T:" + std::to_string(tracked_cones.size());
        cv::putText(frame, stat, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, YELLOW, 2);
    }

    /**
     * @brief 绘制红色锥桶
     */
    inline void drawDetectedRedCones(cv::Mat& frame, bool draw_info = true) {
        const cv::Scalar RED(0, 0, 255), MAGENTA(255, 0, 255), ORANGE(0, 165, 255);

        // 绘制红色补线路径
        if (red_line_points.size() > 1) {
            for (size_t i = 0; i < red_line_points.size() - 1; ++i) {
                cv::line(frame, red_line_points[i], red_line_points[i + 1], MAGENTA, 2);
            }
        }

        for (const auto& cone : detected_red_cones) {
            cv::rectangle(frame, cone.bounding_box, RED, 2);
            cv::circle(frame, cone.center, 8, ORANGE, -1);
            cv::circle(frame, cone.center, 8, RED, 2);

            if (draw_info) {
                std::string id_text = "RED #" + std::to_string(cone.id);
                cv::putText(frame, id_text, cv::Point(cone.bounding_box.x, cone.bounding_box.y - 5),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, RED, 2);
            }
        }
    }

    inline const std::vector<ConeObject>& getCones() { return detected_cones; }
    inline const std::vector<ConeObject>& getRedCones() { return detected_red_cones; }
    inline size_t getConeCount() { return detected_cones.size(); }
    inline size_t getRedConeCount() { return detected_red_cones.size(); }

    inline void printConeInfo() {
        std::cout << "\n=== Cone Detection ===" << std::endl;
        std::cout << "Visible: " << detected_cones.size() << " | Total ID: " << next_cone_id << std::endl;
        for (const auto& cone : detected_cones) {
            std::cout << "ID#" << cone.id << ": (" << cone.center.x << "," << cone.center.y
                     << ") Area:" << static_cast<int>(cone.area) << std::endl;
        }

        // 打印补线路径
        if (line_points.size() > 0) {
            std::cout << "\nLine Path Points (" << line_points.size() << "):" << std::endl;
            for (size_t i = 0; i < line_points.size() && i < 10; ++i) {  // 最多打印前10个点
                std::cout << "  [" << i << "] (" << line_points[i].x << "," << line_points[i].y << ")";
                if ((i + 1) % 3 == 0) std::cout << "\n";
            }
            if (line_points.size() > 10) std::cout << "  ... and " << (line_points.size() - 10) << " more";
            std::cout << "\n";
        }
        std::cout << "======================\n" << std::endl;
    }

    inline void printRedConeInfo() {
        std::cout << "\n=== Red Cone Detection ===" << std::endl;
        std::cout << "Visible: " << detected_red_cones.size() << " | Total ID: " << next_red_cone_id << std::endl;
        for (const auto& cone : detected_red_cones) {
            std::cout << "RED ID#" << cone.id << ": (" << cone.center.x << "," << cone.center.y
                     << ") Area:" << static_cast<int>(cone.area) << std::endl;
        }
        std::cout << "==========================\n" << std::endl;
    }

    inline int getError()
    {
        int error = 0;
        cv::Mat mask = cv::Mat::zeros(frame_size_height, frame_size_width, CV_8UC1);

        // 使用补完点的完整路径来计算误差，提供连续的路径信息用于控制
        if (!line_points.empty() && !red_line_points.empty()) {
            // 绘制黄色锥桶的完整补线路径（包含虚拟点）
            for (int index = 0; index < line_points.size() - 1; index++) {
                cv::line(mask, line_points[index], line_points[index + 1], cv::Scalar(255), 8);
            }
            // 绘制红色锥桶的完整补线路径（包含虚拟点）
            for (int index = 0; index < red_line_points.size() - 1; index++) {
                cv::line(mask, red_line_points[index], red_line_points[index + 1], cv::Scalar(255), 8);
            }
#ifdef _DEBUG
            cv::imshow("complete path mask", mask);
#endif
            // imgQ 里的神秘小数字
            cv::Rect roi_rect(0, (mask.rows / 2 - 90 + 70), mask.cols, (mask.rows / 2.5));
            cv::Mat cropped_image = mask(roi_rect);
            if (!cropped_image.empty()) cv::resize(cropped_image, cropped_image, cv::Size(), 0.5, 0.5);
            else std::cout << "Empty Img" << std::endl;
            error = GuidedImgHandle(cropped_image);
        }
        return error;
    }
}


#endif // !__CONE_DETECTOR_HPP