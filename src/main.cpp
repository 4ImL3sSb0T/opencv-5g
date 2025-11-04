#include <iostream>
#include "config.hpp"
#include "garage.hpp"

int main()
{
    cv::Mat frame;
    if (!Config::load_config("../config/config.json"))
    {
        std::cout << "Error loading config.json" << std::endl;
        return -1;
    }
    auto config = Config::get_config();
    std::string path;
    try {
        path = config["vision"]["path"].get<std::string>();
    } catch (std::exception& e) {
        std::cout << "Error loading value" << std::endl;
        return -1;
    }
    auto cap = cv::VideoCapture(path);
    // auto cap = cv::VideoCapture(1);
    Garage::initGarage();
    
    // 创建HSV滑条窗口并初始化
    cv::namedWindow("HSV Controls", cv::WINDOW_AUTOSIZE);
    int lowH = static_cast<int>(Garage::yellow_low[0]);
    int lowS = static_cast<int>(Garage::yellow_low[1]);
    int lowV = static_cast<int>(Garage::yellow_low[2]);
    int highH = static_cast<int>(Garage::yellow_high[0]);
    int highS = static_cast<int>(Garage::yellow_high[1]);
    int highV = static_cast<int>(Garage::yellow_high[2]);

    cv::createTrackbar("Low H",  "HSV Controls", &lowH, 179);
    cv::createTrackbar("Low S",  "HSV Controls", &lowS, 255);
    cv::createTrackbar("Low V",  "HSV Controls", &lowV, 255);
    cv::createTrackbar("High H", "HSV Controls", &highH, 179);
    cv::createTrackbar("High S", "HSV Controls", &highS, 255);
    cv::createTrackbar("High V", "HSV Controls", &highV, 255);
    
    while (true)
    {
        auto ret = cap.read(frame);
        if (ret != true) break;
        // 读取滑条并同步到Garage阈值（确保low<=high）
        int curLowH  = cv::getTrackbarPos("Low H",  "HSV Controls");
        int curLowS  = cv::getTrackbarPos("Low S",  "HSV Controls");
        int curLowV  = cv::getTrackbarPos("Low V",  "HSV Controls");
        int curHighH = cv::getTrackbarPos("High H", "HSV Controls");
        int curHighS = cv::getTrackbarPos("High S", "HSV Controls");
        int curHighV = cv::getTrackbarPos("High V", "HSV Controls");

        int lH = std::min(curLowH,  curHighH);
        int hH = std::max(curLowH,  curHighH);
        int lS = std::min(curLowS,  curHighS);
        int hS = std::max(curLowS,  curHighS);
        int lV = std::min(curLowV,  curHighV);
        int hV = std::max(curLowV,  curHighV);

        Garage::yellow_low  = cv::Scalar(lH, lS, lV);
        Garage::yellow_high = cv::Scalar(hH, hS, hV);
        Garage::Update(frame);
        
    }
}