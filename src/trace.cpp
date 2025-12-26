#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>
#include "CLI11.hpp"
#include <cmath>
#include <string>

#ifdef _WIN32
#define NOMINMAX  // 防止Windows.h定义min/max宏
#include <windows.h>
#endif

int preprocessMode = 0; // 0 顶帽+OTSU，1 自适应阈值，2 原 image_Q 预处理

int main(int argc, char** argv)
{
#ifdef _WIN32
    // 设置Windows控制台为UTF-8编码，解决中文乱码问题
    SetConsoleOutputCP(CP_UTF8);
#endif

    CLI::App app{"Trace - Image preprocessing and visualization tool"};

    // 定义参数
    int cameraIndex = -1;
    std::string videoPath;
    std::string imagePath;

    // 添加选项
    auto* cameraOpt = app.add_option("-c,--camera", cameraIndex, "Camera device index (e.g., 0, 1)")
        ->check(CLI::NonNegativeNumber);
    auto* videoOpt = app.add_option("-v,--video", videoPath, "Path to video file")
        ->check(CLI::ExistingFile);
    auto* imageOpt = app.add_option("-i,--image", imagePath, "Path to image file")
        ->check(CLI::ExistingFile);

    // 设置互斥：只能选择摄像头、视频或图片其中之一
    cameraOpt->excludes(videoOpt)->excludes(imageOpt);
    videoOpt->excludes(imageOpt);

    // 解析命令行参数
    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &e) {
        return app.exit(e);
    }

    spdlog::info("trace: TUxiang_Init 前半部分调试（预处理+可视化），不含扫线和元素检测");

    // 确定输入源类型
    bool isImageMode = false;
    cv::VideoCapture cap;
    cv::Mat staticImage;

    if (!imagePath.empty()) {
        // 图片模式
        staticImage = cv::imread(imagePath);
        if (staticImage.empty()) {
            spdlog::error("Failed to load image: {}", imagePath);
            return -1;
        }
        isImageMode = true;
        spdlog::info("Input mode: Image file - {}", imagePath);
    }
    else if (!videoPath.empty()) {
        // 视频文件模式
        cap.open(videoPath);
        if (!cap.isOpened()) {
            spdlog::error("Failed to open video file: {}", videoPath);
            return -1;
        }
        spdlog::info("Input mode: Video file - {}", videoPath);
    }
    else if (cameraIndex >= 0) {
        // 摄像头模式
        cap.open(cameraIndex);
        if (!cap.isOpened()) {
            spdlog::error("Failed to open camera index: {}", cameraIndex);
            return -1;
        }
        spdlog::info("Input mode: Camera - index {}", cameraIndex);
    }
    else {
        // 默认：使用摄像头 0
        cap.open(0);
        if (!cap.isOpened()) {
            spdlog::error("Failed to open default camera (index 0)");
            return -1;
        }
        spdlog::info("Input mode: Default camera (index 0)");
    }

    cv::Mat frame;
    while (true)
    {
        if (isImageMode) {
            // 图片模式：重复使用同一张图片
            frame = staticImage.clone();
        } else {
            // 视频/摄像头模式：读取下一帧
            if (!cap.read(frame)) {
                spdlog::info("End of video stream");
                break;
            }
        }

        if (frame.empty())
        {
            continue;
        }

        // -------- TUxiang_Init 前半段：裁剪、缩放、滤波、边缘、Hough --------
        cv::Mat banmachuli = frame.clone();
        cv::resize(banmachuli, banmachuli, cv::Size(), 0.5, 0.5);

        int yStart = frame.rows / 2 - 90 + 70;
        int roiHeight = static_cast<int>(frame.rows / 2.5);
        if (yStart < 0)
        {
            yStart = 0;
        }
        if (yStart + roiHeight > frame.rows)
        {
            roiHeight = frame.rows - yStart;
        }
        cv::Rect roiRect(0, yStart, frame.cols, roiHeight);
        cv::Mat cropped_image = frame(roiRect).clone();

        cv::resize(cropped_image, cropped_image, cv::Size(), 0.5, 0.5);

        cv::Mat hsv_image;
        cv::cvtColor(cropped_image, hsv_image, cv::COLOR_BGR2HSV);

        cv::Mat gray_image;
        cv::cvtColor(cropped_image, gray_image, cv::COLOR_BGR2GRAY);

        cv::Mat ca;
        cv::Mat binary_img;
        if (preprocessMode == 0)   // 顶帽 + OTSU
        {
            cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(17, 17));
            cv::Mat tophat_img;
            cv::morphologyEx(gray_image, tophat_img, cv::MORPH_TOPHAT, element);
            cv::normalize(tophat_img, tophat_img, 0, 255, cv::NORM_MINMAX);

            cv::threshold(tophat_img, binary_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        }
        else if (preprocessMode == 1)   // 自适应阈值
        {
            cv::Mat blur;
            cv::GaussianBlur(gray_image, blur, cv::Size(5, 5), 0);
            cv::adaptiveThreshold(blur, binary_img, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 21, -5);
            cv::medianBlur(binary_img, binary_img, 5);
        }
        else    // 2: 原 image_Q 预处理（双边+高斯+Canny）
        {
            cv::Mat blur;
            cv::bilateralFilter(gray_image, blur, 7, 60, 60);
            cv::GaussianBlur(blur, binary_img, cv::Size(5, 5), 30);
            // 这里 binary_img 实际是高斯后的灰度
        }

        if (preprocessMode == 2)
        {
            cv::Canny(binary_img, ca, 30, 50);
        }
        else
        {
            cv::Canny(binary_img, ca, 50, 150);
        }

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
        cv::Mat dilated_ca;
        cv::dilate(ca, dilated_ca, kernel, cv::Point(-1, -1), 1);

        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(dilated_ca, lines, 1, CV_PI / 180, 70, 25, 5);

        cv::Mat line_image = cv::Mat::zeros(dilated_ca.size(), CV_8UC1);
        std::vector<cv::Vec4i> filtered_lines;

        double min_angle = -90.0;
        double max_angle = -18.0;
        for (const auto &line : lines)
        {
            double angle_rad = std::atan2(line[3] - line[1], line[2] - line[0]);
            double angle_deg = angle_rad * 180.0 / CV_PI;
            if (angle_deg >= min_angle && angle_deg <= max_angle)
            {
                filtered_lines.push_back(line);
            }
        }
        for (const auto &line : filtered_lines)
        {
            cv::line(line_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255), 2, cv::LINE_AA);
        }

        min_angle = 18.0;
        max_angle = 90.0;
        for (const auto &line : lines)
        {
            double angle_rad = std::atan2(line[3] - line[1], line[2] - line[0]);
            double angle_deg = angle_rad * 180.0 / CV_PI;
            if (angle_deg >= min_angle && angle_deg <= max_angle)
            {
                filtered_lines.push_back(line);
            }
        }
        for (size_t i = 0; i < filtered_lines.size(); ++i)
        {
            const cv::Vec4i &line = filtered_lines[i];
            cv::line(line_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255), 2, cv::LINE_AA);
        }

        cv::Mat dilated_ca2;
        cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
        cv::dilate(line_image, dilated_ca2, kernel2, cv::Point(-1, -1), 1);

        // -------- 调试可视化 --------
        cv::imshow("cropped", cropped_image);
        cv::imshow("tophat_binary", binary_img);
        cv::imshow("canny", ca);
        cv::imshow("hough_filtered", dilated_ca2);

        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q')
        {
            break;
        }
        if (key == 'm' || key == 'M')
        {
            preprocessMode = (preprocessMode + 1) % 3; // 0 顶帽, 1 自适应, 2 原 image_Q
            spdlog::info("switch preprocess mode to {}", preprocessMode);
        }
    }

    return 0;
}
