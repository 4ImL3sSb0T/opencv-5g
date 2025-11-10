//
// Created by yuang on 2025/11/9.
//
#include "guided.h"
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

int main(int argv, char** argc) {
    auto capture = cv::VideoCapture("../img/guided_left.mp4");

    Guided::initGuided();
    cv::Mat frame;

    while (capture.read(frame)) {
        if (frame.empty()) {
            spdlog::error("Failed to load image");
            return -1;
        }
        Guided::Update(frame);
        if (cv::waitKey(30) >= 0) break;
    }
    return 0;
}