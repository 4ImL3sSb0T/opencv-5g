//
// Created by yuang on 2025/11/6.
//

#ifndef OPENCV_5G_GUIDED_H
#define OPENCV_5G_GUIDED_H
#include "json.hpp"
#include <opencv2/opencv.hpp>
#include <limits>
#include <algorithm>
#include <string>
#ifdef _DEBUG
#include <spdlog/spdlog.h>
#endif


namespace Guided
{
    struct TrackedCone {
        int id{ -1 };
        cv::Point2f center{ -1.f, -1.f };
        cv::Rect bbox{ 0, 0, 0, 0 };
        bool active{ false };
        int missedFrames{ 0 };
    };

    // HSV 阈值（可根据实际环境在 initGuided 中加载/调整）
    inline auto yellow_lower = cv::Scalar(17, 62, 135);
    inline auto yellow_upper = cv::Scalar(57, 255, 255);
    inline auto red_lower = cv::Scalar(0, 100, 100);
    inline auto red_upper = cv::Scalar(10, 255, 255);

    // 形态学与过滤阈值
    inline int morph_kernel_size = 5;
    inline int morph_iterations = 1;
    inline double min_contour_area = 150.0;         // 过滤细小噪声
    inline double min_aspect_ratio = 0.4;           // w/h 下限（锥桶不应过扁）
    inline double max_aspect_ratio = 1.5;           // w/h 上限（锥桶不应过宽）
    inline int max_missed_frames = 20;              // 允许出屏的最大丢帧数
    inline double reassign_max_distance = 200.0;    // 最近邻重关联最大距离像素

    inline std::vector<std::vector<cv::Point>> yellow_contours;
    inline std::vector<std::vector<cv::Point>> last_yellow_contours;

    inline std::vector<std::vector<cv::Point>> red_contours;
    inline std::vector<std::vector<cv::Point>> last_red_contours;

    // 多目标轨迹池
    inline std::vector<TrackedCone> yellow_tracks;
    inline std::vector<TrackedCone> red_tracks;
    inline int next_yellow_id = 1;
    inline int next_red_id = 1;

    inline cv::Mat preprocessMask(const cv::Mat& mask) {
        cv::Mat processed;
        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_kernel_size, morph_kernel_size));
        // 开运算去噪 + 闭运算填洞
        cv::morphologyEx(mask, processed, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), morph_iterations);
        cv::morphologyEx(processed, processed, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), morph_iterations);
        return processed;
    }

    inline std::vector<std::vector<cv::Point>> filterContours(const std::vector<std::vector<cv::Point>>& contours) {
        std::vector<std::vector<cv::Point>> filtered;
        filtered.reserve(contours.size());
        for (const auto& c : contours) {
            const double area = cv::contourArea(c);
            if (area < min_contour_area) continue;
            const cv::Rect box = cv::boundingRect(c);
            if (box.width <= 0 || box.height <= 0) continue;
            const double aspect = static_cast<double>(box.width) / static_cast<double>(box.height);
            if (aspect < min_aspect_ratio || aspect > max_aspect_ratio) continue;
            filtered.emplace_back(c);
        }
        return filtered;
    }

    inline cv::Point2f contourCenter(const std::vector<cv::Point>& c) {
        const cv::Rect box = cv::boundingRect(c);
        return cv::Point2f(static_cast<float>(box.x + box.width * 0.5f),
                           static_cast<float>(box.y + box.height * 0.5f));
    }

    inline int findNearestContourIndex(const std::vector<std::vector<cv::Point>>& contours, const cv::Point2f& lastCenter, double maxDist) {
        if (contours.empty()) return -1;
        int bestIdx = -1;
        double bestDist = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
            const cv::Point2f c = contourCenter(contours[i]);
            const double d = cv::norm(c - lastCenter);
            if (d < bestDist) {
                bestDist = d;
                bestIdx = i;
            }
        }
        if (bestDist <= maxDist) return bestIdx;
        return -1;
    }

    inline int findLargestContourIndex(const std::vector<std::vector<cv::Point>>& contours) {
        if (contours.empty()) return -1;
        int bestIdx = -1;
        double bestArea = -1.0;
        for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
            const double area = cv::contourArea(contours[i]);
            if (area > bestArea) {
                bestArea = area;
                bestIdx = i;
            }
        }
        return bestIdx;
    }

    inline bool isOutOfFrame(const TrackedCone& t, const cv::Size& size) {
        // 以 bbox 判定是否完全在屏幕外，或中心点越界
        const bool bbox_out =
            (t.bbox.x + t.bbox.width <= 0) || (t.bbox.x >= size.width) ||
            (t.bbox.y + t.bbox.height <= 0) || (t.bbox.y >= size.height);
        const bool center_out =
            (t.center.x < 0.f || t.center.x >= static_cast<float>(size.width) ||
             t.center.y < 0.f || t.center.y >= static_cast<float>(size.height));
        return bbox_out || center_out;
    }

    inline void updateTracksForColor(const std::vector<std::vector<cv::Point>>& contours,
                                     std::vector<TrackedCone>& tracks,
                                     int& next_id) {
        // 预计算检测中心与 bbox
        struct Detection { cv::Point2f center; cv::Rect bbox; int idx; };
        std::vector<Detection> detections;
        detections.reserve(contours.size());
        for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
            Detection d;
            d.center = contourCenter(contours[i]);
            d.bbox = cv::boundingRect(contours[i]);
            d.idx = i;
            detections.emplace_back(d);
        }

        // 标记未匹配集合
        std::vector<bool> det_used(detections.size(), false);
        std::vector<bool> trk_used(tracks.size(), false);

        // 构建所有可能的配对并按距离排序（贪心最近邻匹配）
        struct Pair { int t; int d; double dist; };
        std::vector<Pair> pairs;
        pairs.reserve(tracks.size() * detections.size());
        for (int t = 0; t < static_cast<int>(tracks.size()); ++t) {
            if (!tracks[t].active) continue;
            for (int d = 0; d < static_cast<int>(detections.size()); ++d) {
                const double dist = cv::norm(tracks[t].center - detections[d].center);
                if (dist <= reassign_max_distance) {
                    pairs.push_back({ t, d, dist });
                }
            }
        }
        std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b){ return a.dist < b.dist; });

        // 执行匹配
        for (const auto& p : pairs) {
            if (p.t < 0 || p.t >= static_cast<int>(tracks.size())) continue;
            if (p.d < 0 || p.d >= static_cast<int>(detections.size())) continue;
            if (trk_used[p.t] || det_used[p.d]) continue;
            // 关联
            tracks[p.t].center = detections[p.d].center;
            tracks[p.t].bbox = detections[p.d].bbox;
            tracks[p.t].missedFrames = 0;
            tracks[p.t].active = true;
            trk_used[p.t] = true;
            det_used[p.d] = true;
        }

        // 未匹配的轨迹累积丢帧
        for (int t = 0; t < static_cast<int>(tracks.size()); ++t) {
            if (!tracks[t].active) continue;
            if (!trk_used[t]) {
                tracks[t].missedFrames++;
            }
        }

        // 为未匹配的检测新建轨迹
        for (int d = 0; d < static_cast<int>(detections.size()); ++d) {
            if (det_used[d]) continue;
            TrackedCone trk;
            trk.id = next_id++;
            trk.center = detections[d].center;
            trk.bbox = detections[d].bbox;
            trk.active = true;
            trk.missedFrames = 0;
            tracks.emplace_back(trk);
        }

        // 清理长期丢失的轨迹
        tracks.erase(std::remove_if(tracks.begin(), tracks.end(), [](const TrackedCone& t){
            return t.active && t.missedFrames > max_missed_frames;
        }), tracks.end());
    }

    inline void initGuided() {
        
    }

    inline void processFrame(const cv::Mat& frame) {
        cv::Mat hsv;
        cv::Mat draw_frame = frame.clone();
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::Mat yellow_mask, red_mask;
        cv::inRange(hsv, yellow_lower, yellow_upper, yellow_mask);
        cv::inRange(hsv, red_lower, red_upper, red_mask);

        // 形态学处理
        yellow_mask = preprocessMask(yellow_mask);
        red_mask = preprocessMask(red_mask);

        // 轮廓提取
        cv::findContours(yellow_mask, yellow_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(red_mask, red_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 轮廓过滤
        yellow_contours = filterContours(yellow_contours);
        red_contours = filterContours(red_contours);

        // 多目标跟踪
        updateTracksForColor(yellow_contours, yellow_tracks, next_yellow_id);
        updateTracksForColor(red_contours, red_tracks, next_red_id);

        // 出屏即刻丢弃并标记一次 "OUT"
        auto remove_out = [&](std::vector<TrackedCone>& tracks, const cv::Scalar& color){
            for (auto& t : tracks) {
                if (!t.active) continue;
                if (isOutOfFrame(t, frame.size())) {
                    const int px = std::max(0, std::min(static_cast<int>(t.center.x), frame.cols - 1));
                    const int py = std::max(0, std::min(static_cast<int>(t.center.y), frame.rows - 1));
                    cv::putText(draw_frame, "OUT", cv::Point(px, py),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
                    t.active = false;
                    t.missedFrames = max_missed_frames + 1;
#ifdef _DEBUG
                    spdlog::info("Track {} removed: out of frame", t.id);
#endif
                }
            }
            tracks.erase(std::remove_if(tracks.begin(), tracks.end(),
                                        [](const TrackedCone& t){ return !t.active; }),
                         tracks.end());
        };

        remove_out(yellow_tracks, cv::Scalar(0, 255, 255));
        remove_out(red_tracks, cv::Scalar(0, 0, 255));

        // 可视化
        for (const auto& t : yellow_tracks) {
            if (!t.active || t.missedFrames > max_missed_frames) continue;
            cv::rectangle(draw_frame, t.bbox, cv::Scalar(0, 255, 255), 2);
            cv::circle(draw_frame, t.center, 4, cv::Scalar(0, 255, 255), -1);
            cv::putText(draw_frame, "Y" + std::to_string(t.id), t.center + cv::Point2f(0.f, -8.f),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        }
        for (const auto& t : red_tracks) {
            if (!t.active || t.missedFrames > max_missed_frames) continue;
            cv::rectangle(draw_frame, t.bbox, cv::Scalar(0, 0, 255), 2);
            cv::circle(draw_frame, t.center, 4, cv::Scalar(0, 0, 255), -1);
            cv::putText(draw_frame, "R" + std::to_string(t.id), t.center + cv::Point2f(0.f, -8.f),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        }

        cv::imshow("draw", draw_frame);
        cv::imshow("yellow", yellow_mask);
        cv::imshow("red", red_mask);
    }

    inline void Update(const cv::Mat& frame) {
        processFrame(frame);
    }
}

#endif //OPENCV_5G_GUIDED_H