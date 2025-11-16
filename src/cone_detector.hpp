#ifndef __CONE_DETECTOR_HPP
#define __CONE_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>

namespace ConeDetector {
    // ============ 配置常数 ============
    constexpr int MAX_CONES = 5;                           // 最多识别锥桶数
    constexpr int DEFAULT_MORPH_KERNEL = 5;
    constexpr double DEFAULT_MIN_AREA = 100.0;
    constexpr double DEFAULT_MAX_AREA = 50000.0;
    constexpr double DEFAULT_RATIO_MIN = 0.3;
    constexpr double DEFAULT_RATIO_MAX = 3.0;
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
    inline std::vector<ConeObject> detected_cones;
    inline std::map<int, ConeObject> tracked_cones;
    inline int next_cone_id = 0;
    inline std::vector<cv::Point> line_points;  // 补线后的路径点

    // ============ 核心函数 ============
    inline void initConeDetector(const cv::Scalar& hsv_low, const cv::Scalar& hsv_high,
                                 double min_area = DEFAULT_MIN_AREA, double max_area = DEFAULT_MAX_AREA) {
        detection_params.hsv_low = hsv_low;
        detection_params.hsv_high = hsv_high;
        detection_params.min_area = min_area;
        detection_params.max_area = max_area;
        next_cone_id = 0;
        tracked_cones.clear();
    }

    inline double calculateDistance(const cv::Point& p1, const cv::Point& p2) {
        double dx = p1.x - p2.x, dy = p1.y - p2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    inline int getNextAvailableId() {
        return (next_cone_id >= MAX_CONES) ? -1 : next_cone_id++;
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
        for (size_t i = 0; i < new_detections.size(); ++i) {
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

    /**
     * @brief 计算相邻锥桶间的补线路径点
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

    inline std::vector<ConeObject> detectCones(const cv::Mat& frame) {
        detected_cones.clear();

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
            if (cone.id >= 0 && cone.id < MAX_CONES) {
                valid.push_back(cone);
            }
        }
        detected_cones = valid;
        
        // 计算补线点
        computeLinePath();
        
        return detected_cones;
    }

    /**
     * @brief 获取补线后的路径点
     * @return 路径点列表
     */
    inline const std::vector<cv::Point>& getLinePath() {
        return line_points;
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

    inline const std::vector<ConeObject>& getCones() { return detected_cones; }
    inline size_t getConeCount() { return detected_cones.size(); }

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

} // namespace ConeDetector

#endif // !__CONE_DETECTOR_HPP
