#include <iostream>
#include "config.hpp"
#include "garage.hpp"
#include "cone_detector.hpp"

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

    cv::VideoCapture cap;
    if (path == "video0") {
        cap = cv::VideoCapture(0);
    } else if (path == "video1") {
        cap = cv::VideoCapture(1);
    } else if (path == "video2") {
        cap = cv::VideoCapture(2);
    } else {
        cap = cv::VideoCapture(path);
    }
    
    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }
    Garage::initGarage();
    
    // 从配置文件读取锥桶检测参数（重用已有的 config 对象）
    double cone_min_area = config["vision"]["cone_detection"]["min_area"].get<double>();
    double cone_max_area = config["vision"]["cone_detection"]["max_area"].get<double>();
    double tracking_distance = config["vision"]["cone_detection"]["tracking_distance_threshold"].get<double>();
    int max_disappeared = config["vision"]["cone_detection"]["max_disappeared_frames"].get<int>();
    
    // 初始化锥桶检测器
    ConeDetector::initConeDetector(Garage::yellow_low, Garage::yellow_high, cone_min_area, cone_max_area);
    ConeDetector::detection_params.tracking_distance_threshold = tracking_distance;
    ConeDetector::detection_params.max_disappeared_frames = max_disappeared;
    
    std::cout << "Cone Detection Initialized:" << std::endl;
    std::cout << "  Min Area: " << cone_min_area << " px²" << std::endl;
    std::cout << "  Max Area: " << cone_max_area << " px²" << std::endl;
    std::cout << "  Tracking Distance Threshold: " << tracking_distance << " px" << std::endl;
    std::cout << "  Max Disappeared Frames: " << max_disappeared << std::endl;
    
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
    
    bool isPaused = false;
    
    // 创建一个大窗口专门用于显示锥桶检测结果
    cv::namedWindow("Cone Detection", cv::WINDOW_NORMAL);
    cv::resizeWindow("Cone Detection", 800, 600);
    
    while (true)
    {
        // 如果未暂停，读取下一帧
        if (!isPaused) {
            auto ret = cap.read(frame);
            if (ret != true) break;
        }
        
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
        
        // 更新锥桶检测器的HSV参数
        ConeDetector::detection_params.hsv_low = cv::Scalar(lH, lS, lV);
        ConeDetector::detection_params.hsv_high = cv::Scalar(hH, hS, hV);
        
        // 执行锥桶检测
        ConeDetector::detectCones(frame);
        
        // 创建专用于显示锥桶检测的副本
        auto cone_display = frame.clone();
        ConeDetector::drawDetectedCones(cone_display, true);
        
        // 显示锥桶检测结果窗口（大窗口，便于观察）
        cv::imshow("Cone Detection", cone_display);
        
        // 也在原始Frame副本上绘制用于Garage处理
        auto display_frame = frame.clone();
        ConeDetector::drawDetectedCones(display_frame, false);  // 不显示详细信息，避免干扰Garage检测
        
        Garage::Update(frame);
        
        // 按键处理 - 暂停时使用较短延迟以保证响应性，播放时使用配置的延迟
        auto waitKeyTime = isPaused ? 10 : Garage::wait_time;
        auto key = cv::waitKey(waitKeyTime) & 0xFF;
        if (key == 'q' || key == 'Q' || key == 27) {
            // 'q' 或 ESC 退出
            std::cout << "Exiting..." << std::endl;
            break;
        } else if (key == ' ') {
            // 空格键暂停/继续
            isPaused = !isPaused;
            if (isPaused) {
                std::cout << "Video PAUSED - Press SPACE to resume, 'q' to quit" << std::endl;
            } else {
                std::cout << "Video RESUMED" << std::endl;
            }
        } else if (key == 'p' || key == 'P') {
            // 'p' 键打印当前帧的锥桶检测结果
            ConeDetector::printConeInfo();
        }
    }
    
    cap.release();
    cv::destroyAllWindows();
}