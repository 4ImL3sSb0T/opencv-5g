#ifndef CONE_DETECTOR_HPP
#define CONE_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include "params.hpp"  // ⭐ 引入参数配置

class ConeDetector {
public:
    struct Cone {
        cv::Point2f position;
        float area;
        int side; // 0=左, 1=右
    };

    ConeDetector() {
        lower_yellow_ = cv::Scalar(CONE_HSV_H_MIN, CONE_HSV_S_MIN, CONE_HSV_V_MIN);
        upper_yellow_ = cv::Scalar(CONE_HSV_H_MAX, CONE_HSV_S_MAX, CONE_HSV_V_MAX);
        min_area_ = CONE_AREA_MIN;
        max_area_ = CONE_AREA_MAX;
    }

    // 动态设置HSV范围（用于调参）
    void setHSVRange(cv::Scalar lower, cv::Scalar upper) {
        lower_yellow_ = lower;
        upper_yellow_ = upper;
    }

    // 动态设置面积范围（用于调参）
    void setAreaRange(double min, double max) {
        min_area_ = min;
        max_area_ = max;
    }

    // 检测锥桶
    std::vector<Cone> detect(const cv::Mat& frame) {
        std::vector<Cone> cones;
        if (frame.empty()) return cones;

        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, lower_yellow_, upper_yellow_, mask);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        int center_x = frame.cols / 2;
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < min_area_ || area > max_area_) continue;

            cv::Moments m = cv::moments(contour);
            if (m.m00 == 0) continue;

            Cone cone;
            cone.position.x = m.m10 / m.m00;
            cone.position.y = m.m01 / m.m00;
            cone.area = area;
            cone.side = (cone.position.x < center_x) ? 0 : 1;
            cones.push_back(cone);
        }

        // 按Y坐标排序(从近到远)
        std::sort(cones.begin(), cones.end(),
                  [](const Cone& a, const Cone& b) { return a.position.y > b.position.y; });
        return cones;
    }

    // ⭐⭐⭐ 核心函数：根据锥桶计算虚拟中线的error值 ⭐⭐⭐
    int calculateError(const std::vector<Cone>& cones, int frame_width, int frame_height, int arrow_direction) {
        if (cones.empty()) {
            return frame_width / 2; // 没有锥桶，返回中心
        }

        // 分离左右侧锥桶
        std::vector<Cone> left_cones, right_cones;
        for (const auto& cone : cones) {
            if (cone.side == 0) left_cones.push_back(cone);
            else right_cones.push_back(cone);
        }

        // 计算检测行（图像下方1/3处）
        int detect_y = frame_height * 2 / 3;
        
        // 计算左右边界点
        int left_x = 0;
        int right_x = frame_width;

        // 左侧边界拟合
        if (left_cones.size() >= 2) {
            left_x = fitLineAtY(left_cones, detect_y);
        } else if (left_cones.size() == 1) {
            left_x = left_cones[0].position.x;
        }

        // 右侧边界拟合
        if (right_cones.size() >= 2) {
            right_x = fitLineAtY(right_cones, detect_y);
        } else if (right_cones.size() == 1) {
            right_x = right_cones[0].position.x;
        }

        // 计算虚拟中线
        int virtual_center = (left_x + right_x) / 2;
        
        // 根据换道方向偏移中线
        int offset = VIRTUAL_LINE_OFFSET;
        if (arrow_direction == 1) { // 左换道
            virtual_center -= offset; // 向左偏移，避开左侧锥桶
        } else if (arrow_direction == 2) { // 右换道
            virtual_center += offset; // 向右偏移，避开右侧锥桶
        }

        std::cout << "锥桶补线: 左边界=" << left_x 
                  << " 右边界=" << right_x 
                  << " 虚拟中线=" << virtual_center 
                  << " 偏移=" << (arrow_direction == 1 ? "-" : "+") << offset << std::endl;

        return virtual_center;
    }

    // 可视化调试
    void drawCones(cv::Mat& frame, const std::vector<Cone>& cones) {
        for (const auto& cone : cones) {
            cv::circle(frame, cone.position, 8, cv::Scalar(0, 255, 255), -1);
            cv::putText(frame, 
                        (cone.side == 0 ? "L" : "R"),
                        cv::Point(cone.position.x + 10, cone.position.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }
    }

    void drawVirtualLine(cv::Mat& frame, const std::vector<Cone>& cones, int arrow_direction) {
        if (cones.empty()) return;

        // 分离左右侧锥桶
        std::vector<Cone> left_cones, right_cones;
        for (const auto& cone : cones) {
            if (cone.side == 0) left_cones.push_back(cone);
            else right_cones.push_back(cone);
        }

        // 绘制左侧边界线
        if (left_cones.size() >= 2) {
            for (size_t i = 0; i < left_cones.size() - 1; i++) {
                cv::line(frame, left_cones[i].position, left_cones[i+1].position, 
                         cv::Scalar(255, 0, 0), 2);
            }
        }

        // 绘制右侧边界线
        if (right_cones.size() >= 2) {
            for (size_t i = 0; i < right_cones.size() - 1; i++) {
                cv::line(frame, right_cones[i].position, right_cones[i+1].position, 
                         cv::Scalar(0, 0, 255), 2);
            }
        }

        // 绘制虚拟中线
        int detect_y = frame.rows * 2 / 3;
        int virtual_center = calculateError(cones, frame.cols, frame.rows, arrow_direction);
        cv::circle(frame, cv::Point(virtual_center, detect_y), 10, cv::Scalar(0, 255, 0), -1);
        cv::line(frame, cv::Point(virtual_center, detect_y), 
                 cv::Point(virtual_center, frame.rows), cv::Scalar(0, 255, 0), 2);
    }

private:
    // 拟合直线，返回在指定Y坐标处的X值
    int fitLineAtY(const std::vector<Cone>& cones, int y) {
        if (cones.empty()) return 0;
        if (cones.size() == 1) return cones[0].position.x;

        // 使用最近的两个锥桶进行线性插值
        const Cone& c1 = cones[0];
        const Cone& c2 = cones[1];

        float k = (c2.position.x - c1.position.x) / (c2.position.y - c1.position.y);
        float x = c1.position.x + k * (y - c1.position.y);

        return static_cast<int>(x);
    }

    cv::Scalar lower_yellow_;
    cv::Scalar upper_yellow_;
    double min_area_;
    double max_area_;
};

#endif // CONE_DETECTOR_HPP
