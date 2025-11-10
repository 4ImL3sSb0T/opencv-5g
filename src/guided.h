//
// Created by yuang on 2025/11/6.
//

#ifndef OPENCV_5G_GUIDED_H
#define OPENCV_5G_GUIDED_H
#include "json.hpp"
#include <opencv2/opencv.hpp>
#ifdef _DEBUG
#include <spdlog/spdlog.h>
#endif


namespace Guided
{
    inline auto yellow_lower = cv::Scalar(20, 100, 100);
    inline auto yellow_upper = cv::Scalar(30, 255, 255);
    inline auto red_lower = cv::Scalar(0, 100, 100);
    inline auto red_upper = cv::Scalar(10, 255, 255);

    inline std::vector<std::vector<cv::Point>> yellow_contours;
    inline std::vector<std::vector<cv::Point>> last_yellow_contours;

    inline std::vector<std::vector<cv::Point>> red_contours;
    inline std::vector<std::vector<cv::Point>> last_red_contours;

    inline void initGuided() {
        
    }

    inline void processFrame(const cv::Mat& frame) {
        cv::Mat hsv;
        cv::Mat draw_frame = frame.clone();
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::Mat yellow_mask, red_mask;
        cv::inRange(hsv, yellow_lower, yellow_upper, yellow_mask);
        cv::inRange(hsv, red_lower, red_upper, red_mask);


        cv::findContours(yellow_mask, yellow_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(red_mask, red_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : yellow_contours) {
            const cv::Rect bounding_box = cv::boundingRect(contour);
            cv::rectangle(draw_frame, bounding_box, cv::Scalar(0, 255, 255), 2);
        }

        cv::imshow("draw", draw_frame);
        cv::imshow("yellow", yellow_mask);
    }

    inline void Update(const cv::Mat& frame) {
        processFrame(frame);
    }
}

#endif //OPENCV_5G_GUIDED_H