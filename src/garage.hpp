#ifndef __GARAGE_HPP
#define __GARAGE_HPP

#include "config.hpp"
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <cmath>

namespace Garage
{
    enum Direction
    {
        LEFT,
        RIGHT,
        STRAIGHT,
    };

    enum State
    {
        SEARCHING,
        APPROACHING,
        PARKED,
    };

    inline cv::Scalar yellowLower(13, 69, 55);
    inline cv::Scalar yellowUpper(77, 255, 255);

    inline Direction currentDirection = STRAIGHT;
    inline State currentState = SEARCHING;

    inline int line_threshold = 50;
    inline double min_line_length = 30;
    inline double max_line_gap = 10;

    inline double horizontal_angle_threshold = 10.0;
    inline double vertical_angle_threshold = 10.0;

    inline double canny_threshold1 = 50;
    inline double canny_threshold2 = 150;

    inline int wait_time = 30;

    // inline std::pair<double, cv::Point> car_left_line;
    // inline std::pair<double, cv::Point> car_right_line;
    // inline std::pair<double, cv::Point> car_top_line;
    // inline std::pair<double, cv::Point> car_bottom_line;

    inline std::deque<std::pair<double, cv::Point2f>> mid_lines;
    inline std::deque<std::pair<double, cv::Point2f>> top_lines;
    inline std::deque<std::pair<double, cv::Point2f>> button_lines;

    inline std::pair<double, cv::Point2f> mid_line;
    inline std::pair<double, cv::Point2f> top_line;
    inline std::pair<double, cv::Point2f> button_line;

    // 滑动窗口滤波：固定窗口大小为5
    inline constexpr size_t window_size = 5;

    inline bool initGarage() {
        const auto ret = Config::load_config("../config/config.json");
        std::cout << Config::get_config();
        try {
            auto cfg = Config::get_config();

            line_threshold = cfg["vision"]["garage"]["line_detection"]["line_threshold"].get<int>();
            min_line_length = cfg["vision"]["garage"]["line_detection"]["min_line_length"].get<double>();
            max_line_gap = cfg["vision"]["garage"]["line_detection"]["max_line_gap"].get<double>();

            horizontal_angle_threshold = cfg["vision"]["garage"]["angle"]["horizontal_angle_threshold"].get<double>();
            vertical_angle_threshold = cfg["vision"]["garage"]["angle"]["vertical_angle_threshold"].get<double>();

            canny_threshold1 = cfg["vision"]["garage"]["canny"]["threshold1"].get<double>();
            canny_threshold2 = cfg["vision"]["garage"]["canny"]["threshold2"].get<double>();

            wait_time = cfg["vision"]["wait_time"].get<int>();

            std::cout << "line_threshold " << line_threshold << std::endl;
            std::cout << "min_line_length " << min_line_length << std::endl;
            std::cout << "max_line_gap " << max_line_gap << std::endl;

            std::cout << "horizontal_angle_threshold " << horizontal_angle_threshold << std::endl;
            std::cout << "vertical_angle_threshold " << vertical_angle_threshold << std::endl;

            std::cout << "canny_threshold1 " << canny_threshold1 << std::endl;
            std::cout << "canny_threshold2 " << canny_threshold2 << std::endl;
        }
        catch (const std::exception& e) {
            spdlog::warn("Config Error: {}", e.what());
        }

        return ret;
    }

    inline Direction detectDirection(const cv::Mat& frame) {
        // 方向检测逻辑
        return STRAIGHT;
    }

    inline void drawLine(const cv::Mat& draw_frame, const std::pair<double, cv::Point2f> line, const cv::Scalar& color = cv::Scalar(0, 255, 0)) {
        const double theta = line.first * CV_PI / 180.0f;
        const double dx = std::cosf(theta);
        const double dy = std::sinf(theta);

        constexpr double length = 3000.f;

        cv::Point2f p1(line.second.x + length * dx, line.second.y - length * dy);
        cv::Point2f p2(line.second.x - length * dx, line.second.y + length * dy);

        cv::line(draw_frame, p1, p2, color, 2);
    }

    inline void processGarageImage(const cv::Mat& frame) {
        // 车库图像处理逻辑
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::Mat yellow_mask;
        cv::inRange(hsv, yellowLower, yellowUpper, yellow_mask);

        cv::Mat opening;
        cv::morphologyEx(yellow_mask, opening, cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

        cv::Mat edges;
        cv::Canny(opening, edges, canny_threshold1, canny_threshold2);

        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edges, lines, 1, CV_PI / 180, line_threshold, min_line_length,
                        max_line_gap);

        cv::Mat draw_frame = frame.clone();

        std::vector<std::pair<double, cv::Point>> horizontal_lines;
        std::vector<std::pair<double, cv::Point>> vertical_lines;

        for (auto& line : lines) {
            auto angle = atan2(line[3] - line[1], line[2] - line[0]) * 180.0 / CV_PI;
            cv::Point mid_point((line[0] + line[2]) / 2, (line[1] + line[3]) / 2);

            double abs_angle = std::abs(angle);

            // 判断是否为水平线 (接近 0° 或 180°)
            if (abs_angle < horizontal_angle_threshold ||
                abs_angle > (180 - horizontal_angle_threshold)) {
                horizontal_lines.emplace_back(angle, mid_point);
                cv::line(draw_frame, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]),
                         cv::Scalar(0, 0, 255), 2);
            }
            // 判断是否为垂直线 (接近 90°)
            if (std::abs(abs_angle - 90) < vertical_angle_threshold) {
                vertical_lines.emplace_back(angle, mid_point);

                cv::line(draw_frame, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]),
                         cv::Scalar(0, 255, 255), 2);
            }

            // cv::putText(draw_frame, std::to_string(abs_angle) + "abs", mid_point, cv::FONT_HERSHEY_SIMPLEX, 0.5,
            // 			cv::Scalar(255, 0, 0), 1);
        }
        // 中点位置排序
        // std::ranges::sort(horizontal_lines,
        //                   [](const std::pair<double, cv::Point> &a, const std::pair<double, cv::Point> &b) {
        // 	                  const auto mid_pos_y_a = (a.second.y + a.second.y) / 2;
        // 	                  const auto mid_pos_y_b = (b.second.y + b.second.y) / 2;
        // 	                  return mid_pos_y_a < mid_pos_y_b;
        //                   });
        //
        // std::ranges::sort(vertical_lines,
        //                   [](const std::pair<double, cv::Point> &a, const std::pair<double, cv::Point> &b) {
        // 	                  const auto mid_pos_x_a = (a.second.x + a.second.x) / 2;
        // 	                  const auto mid_pos_x_b = (b.second.x + b.second.x) / 2;
        // 	                  return mid_pos_x_a < mid_pos_x_b;
        //                   });
        cv::Point2f horizontal_sum_pos = {0, 0};

        cv::Point2f vertical_sum_pos = {0, 0};

        if (!horizontal_lines.empty()) {
            double horizontal_sum_k = 0.0f;
            for (const auto& [fst, snd] : horizontal_lines) {
                horizontal_sum_pos.x += snd.x;
                horizontal_sum_pos.y += snd.y;
                horizontal_sum_k += std::abs(fst);
            }
            cv::Point2f horizontal_avg_pos = {
                horizontal_sum_pos.x / horizontal_lines.size(), horizontal_sum_pos.y / horizontal_lines.size()
            };
            auto horizontal_avg_k = horizontal_sum_k / horizontal_lines.size();

            // TODO: 分为车库上端和下端
            drawLine(draw_frame, std::make_pair(horizontal_avg_k, horizontal_avg_pos));
        }

        if (!vertical_lines.empty()) {
            // 位置平均
            double sum_sin2 = 0.0;
            double sum_cos2 = 0.0;
            for (const auto& [fst, snd] : vertical_lines) {
                vertical_sum_pos.x += snd.x;
                vertical_sum_pos.y += snd.y;

                // 将角度规范到 [0, 180)
                double theta_mod = std::fmod(fst + 180.0, 180.0);
                double theta_rad = theta_mod * CV_PI / 180.0;
                // 对无向直线，用 2θ 做圆形均值，避免 90° 左右跳变
                sum_sin2 += std::sin(2.0 * theta_rad);
                sum_cos2 += std::cos(2.0 * theta_rad);

                cv::circle(draw_frame, snd, 5, cv::Scalar(255, 255, 0), -1);
            }
            cv::Point2f vertical_avg_pos = {
                vertical_sum_pos.x / vertical_lines.size(), vertical_sum_pos.y / vertical_lines.size()
            };

            // 计算该帧的垂直角度（单位：度，范围 [0,180)）
            double mean2 = std::atan2(sum_sin2, sum_cos2); // 范围 (-pi, pi]
            if (mean2 < 0) mean2 += 2.0 * CV_PI;           // 转到 [0, 2pi)
            double mean_theta = 0.5 * mean2;               // 回退到 θ 空间，范围 [0, pi)
            double vertical_avg_k = mean_theta * 180.0 / CV_PI;

            cv::putText(draw_frame, "Pos" + std::to_string(vertical_avg_pos.x) + "-" + std::to_string(vertical_avg_pos.y),
            vertical_avg_pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0));
            cv::circle(draw_frame, vertical_avg_pos, 5, cv::Scalar(0, 255, 255), -1);

            if (mid_lines.size() >= window_size) {
                mid_lines.pop_front();
            }
            // 存入本帧的垂直角度（已是 [0,180) 的无向角）与位置
            mid_lines.emplace_back(vertical_avg_k, vertical_avg_pos);
        }
        
        // 计算滑动均值并绘制（即使当前帧没有检测到垂直线，也能显示之前的结果）
        if (!mid_lines.empty()) {
            // 对角度再次做圆形均值平滑（2θ 法），避免跨 0/180 跳变
            double sum_sin2 = 0.0;
            double sum_cos2 = 0.0;
            cv::Point2f sum_pos(0.0f, 0.0f);

            for (const auto& [angle_deg, pos] : mid_lines) {
                double theta_rad = angle_deg * CV_PI / 180.0;
                sum_sin2 += std::sin(2.0 * theta_rad);
                sum_cos2 += std::cos(2.0 * theta_rad);
                sum_pos.x += pos.x;
                sum_pos.y += pos.y;
            }

            double mean2 = std::atan2(sum_sin2, sum_cos2);
            if (mean2 < 0) mean2 += 2.0 * CV_PI;
            double mean_theta = 0.5 * mean2; // [0, pi)

            mid_line.first = mean_theta * 180.0 / CV_PI; // 稳定的无向垂直角
            const size_t count = mid_lines.size();
            mid_line.second.x = sum_pos.x / count;
            mid_line.second.y = sum_pos.y / count;

            cv::circle(draw_frame, mid_line.second, 5, cv::Scalar(0, 0, 255), -1);
            drawLine(draw_frame, mid_line, cv::Scalar(255, 0, 255));
        }
#ifdef _DEBUG
        cv::imshow("Garage", draw_frame);
        cv::imshow("Edges", edges);
#endif

    }


    inline void Update(const cv::Mat& frame) {
        currentDirection = detectDirection(frame);
        processGarageImage(frame);
        cv::waitKey(wait_time);

        switch (currentState) {
        case SEARCHING:
            /* code */
            break;
        case APPROACHING:
            switch (currentDirection) {
            case LEFT:
                /* code */
                break;
            case RIGHT:
                /* code */
                break;
            case STRAIGHT:
                /* code */
                break;

            default:
                break;
            }
            break;

        case PARKED:
            /* code */
            break;
        default:
            break;
        }
    }
} // namespace Garage

#endif // !__GARAGE_HPP
