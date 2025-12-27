#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>
#include "CLI11.hpp"
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

#ifdef _WIN32
#define NOMINMAX  // 防止Windows.h定义min/max宏
#include <windows.h>
#endif

// ==================== 循迹相关类型定义和常量 ====================
#define uint8  unsigned char
#define uint16 unsigned int
#define int8   char
#define int16  int
#define int32  long
#define ROW  76  // 96 - 20
#define COL 320
#define MIDVALUE 160  // COL/2

// ==================== 全局变量 ====================
int preprocessMode = 0; // 0 顶帽+OTSU，1 自适应阈值，2 原 image_Q 预处理，3 HSV阈值
int edgeSearchMode = 0;  // 0 原始算法（找到第一个边界就停止），1 改进算法（多候选边界）

// HSV 阈值参数
int hMin = 0, hMax = 180;
int sMin = 0, sMax = 60;
int vMin = 160, vMax = 255;

// 循迹相关数据结构
int16  Left_Line[ROW + 2], Right_Line[ROW + 2];       // 左右边界
int16  Mid_Line[ROW + 2];                             // 赛道中线
int16  Left_Add_Line[ROW + 2], Right_Add_Line[ROW + 2];   // 左右边界补线数据
int16  Left_Add_Flag[ROW + 2], Right_Add_Flag[ROW + 2];   // 左右边界补线标志
int16  Road_Width_Real[ROW + 2];                      // 实际赛道宽度
int16  Road_Width_Add[ROW + 2];                       // 补线赛道宽度
int16  Line_Count;                                     // 记录成功识别到的赛道行数
int16  Interpolated_Lines[ROW + 2];                    // 插值后的数组

// ==================== 辅助函数 ====================

/**
 * @brief 限幅保护函数
 * @param num 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 限制后的值
 */
int16 Limit_Protect(int16 num, int32 min, int32 max)
{
    if (num >= max) return max;
    else if (num <= min) return min;
    else return num;
}

/**
 * @brief 首行处理 - 初始化虚拟首行中点
 * @return 0 表示成功
 */
int16 First_Line_Handle()
{
    Mid_Line[ROW + 1] = Mid_Line[ROW - 1];
    if (Mid_Line[ROW + 1] >= COL - 40 || Mid_Line[ROW + 1] <= 40)
    {
        Mid_Line[ROW + 1] = COL / 2;
    }
    return 0;
}

/**
 * @brief 边界搜索函数（原始算法）- 找到第一个边界就停止
 * @param i 当前行
 * @param data 二值化图像
 * @param Mid 上一行中点
 * @param Left_Min 左边界最小值
 * @param Right_Max 右边界最大值
 */
void Edge_Search_Mid_Original(int16 i, const cv::Mat& data, int16 Mid, int16 Left_Min, int16 Right_Max)
{
    int16 j;
    const int16 N = 6;  // 前N行丢线特殊处理

    Left_Add_Flag[i] = 1;   // 初始化补线标志为1（需要补线）
    Right_Add_Flag[i] = 1;

    Right_Line[i] = Right_Max;
    Left_Line[i] = Left_Min;

    // 左边线搜索 - 原始算法：找到第一个边界就break
    for (j = Mid; j >= 10; j -= 4)
    {
        // 边界检查，防止越界
        if (j < 8) break;

        if ((data.at<uchar>(i, j) < 100) &&
            (data.at<uchar>(i, j-4) < 100) &&
            (data.at<uchar>(i, j-8) > 100))
        {
            if (j >= COL/2 + 50)
            {
                j = j - 30;  // 跳过可能的干扰
            }
            else
            {
                Left_Add_Flag[i] = 0;
                Left_Line[i] = j;
                Left_Add_Line[i] = j;
                break;  // 找到第一个边界就退出
            }
        }
    }

    // 右边线搜索 - 原始算法：找到第一个边界就break
    for (j = Mid; j <= COL - 10; j += 4)
    {
        // 边界检查，防止越界
        if (j > COL - 9) break;

        if ((data.at<uchar>(i, j) < 100) &&
            (data.at<uchar>(i, j+4) < 100) &&
            (data.at<uchar>(i, j+8) > 100))
        {
            if (j <= COL/2 - 50)
            {
                j = j + 30;  // 跳过可能的干扰
            }
            else
            {
                Right_Add_Flag[i] = 0;
                Right_Line[i] = j;
                Right_Add_Line[i] = j;
                break;  // 找到第一个边界就退出
            }
        }
    }

    // 左边界补线处理
    if (Left_Add_Flag[i])
    {
        if (i >= ROW - N)
            Left_Add_Line[i] = Right_Line[ROW - 1] - 200;
        else
            Left_Add_Line[i] = Right_Add_Line[i + 2] - 200;
    }

    // 右边界补线处理
    if (Right_Add_Flag[i])
    {
        if (i >= ROW - N)
            Right_Add_Line[i] = Left_Line[ROW - 1] + 200;
        else
            Right_Add_Line[i] = Left_Add_Line[i + 2] + 200;
    }

    Road_Width_Real[i] = Right_Line[i] - Left_Line[i];
    Road_Width_Add[i] = Right_Add_Line[i] - Left_Add_Line[i];
    Mid_Line[i] = (Right_Add_Line[i] + Left_Add_Line[i]) / 2;
}

/**
 * @brief 边界搜索函数（改进算法）- 收集所有候选边界，选择最优
 * @param i 当前行
 * @param data 二值化图像
 * @param Mid 上一行中点
 * @param Left_Min 左边界最小值
 * @param Right_Max 右边界最大值
 */
void Edge_Search_Mid_Improved(int16 i, const cv::Mat& data, int16 Mid, int16 Left_Min, int16 Right_Max)
{
    int16 j;
    const int16 N = 6;

    Left_Add_Flag[i] = 1;
    Right_Add_Flag[i] = 1;

    Right_Line[i] = Right_Max;
    Left_Line[i] = Left_Min;

    std::vector<int16> left_candidates;   // 左边界候选点
    std::vector<int16> right_candidates;  // 右边界候选点

    // 左边线搜索 - 改进算法：收集所有候选点
    for (j = Mid; j >= 10; j -= 4)
    {
        // 边界检查，防止越界
        if (j < 8) break;

        if ((data.at<uchar>(i, j) < 100) &&
            (data.at<uchar>(i, j-4) < 100) &&
            (data.at<uchar>(i, j-8) > 100))
        {
            if (j >= COL/2 + 50)
            {
                j = j - 30;
                continue;  // 跳过干扰但继续搜索
            }
            else
            {
                left_candidates.push_back(j);  // 收集候选点
                continue;  // 继续搜索更左边的边界
            }
        }
    }

    // 右边线搜索 - 改进算法：收集所有候选点
    for (j = Mid; j <= COL - 10; j += 4)
    {
        // 边界检查，防止越界
        if (j > COL - 9) break;

        if ((data.at<uchar>(i, j) < 100) &&
            (data.at<uchar>(i, j+4) < 100) &&
            (data.at<uchar>(i, j+8) > 100))
        {
            if (j <= COL/2 - 50)
            {
                j = j + 30;
                continue;  // 跳过干扰但继续搜索
            }
            else
            {
                right_candidates.push_back(j);  // 收集候选点
                continue;  // 继续搜索更右边的边界
            }
        }
    }

    // 从候选点中选择最优边界
    if (!left_candidates.empty())
    {
        // 找到最左边的边界（数值最小的）
        int16 best_left = *std::min_element(left_candidates.begin(), left_candidates.end());
        Left_Add_Flag[i] = 0;
        Left_Line[i] = best_left;
        Left_Add_Line[i] = best_left;
    }

    if (!right_candidates.empty())
    {
        // 找到最右边的边界（数值最大的）
        int16 best_right = *std::max_element(right_candidates.begin(), right_candidates.end());
        Right_Add_Flag[i] = 0;
        Right_Line[i] = best_right;
        Right_Add_Line[i] = best_right;
    }

    // 左边界补线处理
    if (Left_Add_Flag[i])
    {
        if (i >= ROW - N)
            Left_Add_Line[i] = Right_Line[ROW - 1] - 200;
        else
            Left_Add_Line[i] = Right_Add_Line[i + 2] - 200;
    }

    // 右边界补线处理
    if (Right_Add_Flag[i])
    {
        if (i >= ROW - N)
            Right_Add_Line[i] = Left_Line[ROW - 1] + 200;
        else
            Right_Add_Line[i] = Left_Add_Line[i + 2] + 200;
    }

    Road_Width_Real[i] = Right_Line[i] - Left_Line[i];
    Road_Width_Add[i] = Right_Add_Line[i] - Left_Add_Line[i];
    Mid_Line[i] = (Right_Add_Line[i] + Left_Add_Line[i]) / 2;
}

/**
 * @brief 中线线性插值
 */
void LinearInterpolation()
{
    for (int i = ROW - 1; i >= 9; i -= 2)
    {
        Interpolated_Lines[i] = (Mid_Line[i] + Mid_Line[i - 2]) / 2;
        Mid_Line[i - 2] = Interpolated_Lines[i];
    }
}

/**
 * @brief 中线修复
 */
void Mid_Line_Repair()
{
    for (int i = ROW - 1; i >= 9; i -= 2)
    {
        Mid_Line[i] = (Right_Add_Line[i] + Left_Add_Line[i]) / 2;
    }
}

/**
 * @brief 图像处理主函数
 * @param data 二值化后的图像
 * @return 当前误差值
 */
int Image_Handle(const cv::Mat& data)
{
    Line_Count = 0;

    // 初始化边界标志
    for (int i = ROW - 1; i >= 9; i -= 2)
    {
        Left_Add_Flag[i] = 1;
        Right_Add_Flag[i] = 1;
    }

    // 首行特殊处理
    First_Line_Handle();

    // 处理所有行
    for (int i = ROW - 1; i >= 9; i -= 2)
    {
        Line_Count = i;

        // 根据模式选择边界搜索算法
        if (edgeSearchMode == 0)
        {
            Edge_Search_Mid_Original(i, data, Mid_Line[i + 2], 1, COL - 1);
        }
        else
        {
            Edge_Search_Mid_Improved(i, data, Mid_Line[i + 2], 1, COL - 1);
        }
    }

    // 中线线性插值
    LinearInterpolation();

    return 0;
}

/**
 * @brief 绘制循迹结果到图像上
 * @param display 显示图像
 * @param data 二值化图像
 */
void Draw_Track_Result(cv::Mat& display, const cv::Mat& data)
{
    // 如果display是灰度图，转换为彩色以便绘制彩色线条
    if (display.channels() == 1)
    {
        cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);
    }

    for (int i = ROW - 1; i >= 9; i -= 2)
    {
        // 绘制左边界（绿色）
        cv::Point left_pt(Left_Add_Line[i], i);
        cv::circle(display, left_pt, 2, cv::Scalar(0, 255, 0), -1);

        // 绘制右边界（蓝色）
        cv::Point right_pt(Right_Add_Line[i], i);
        cv::circle(display, right_pt, 2, cv::Scalar(255, 0, 0), -1);

        // 绘制中线（红色）
        cv::Point mid_pt(Mid_Line[i], i);
        cv::circle(display, mid_pt, 2, cv::Scalar(0, 0, 255), -1);
    }

    // 在图像上显示当前模式信息（更大更明显）
    std::string mode_text = "Edge Mode: " + std::string(edgeSearchMode == 0 ? "Original" : "Improved");

    // 添加黑色背景，让文字更清晰
    cv::rectangle(display, cv::Point(5, 5), cv::Point(300, 35), cv::Scalar(0, 0, 0), -1);

    // 根据模式使用不同颜色
    cv::Scalar text_color = edgeSearchMode == 0 ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);  // 原始=红色, 改进=绿色

    cv::putText(display, mode_text, cv::Point(10, 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2);
}

// ==================== 主函数 ====================
int main(int argc, char** argv)
{
#ifdef _WIN32
    // 设置Windows控制台为UTF-8编码，解决中文乱码问题
    SetConsoleOutputCP(CP_UTF8);
#endif

    CLI::App app{"Trace - Image preprocessing and line tracking visualization tool"};

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

    spdlog::info("Trace: 图像预处理 + 循迹边界检测 + 可视化调试工具");
    spdlog::info("按键说明:");
    spdlog::info("  M: 切换预处理模式 (0:顶帽+OTSU, 1:自适应阈值, 2:Shadow算法, 3:HSV阈值)");
    spdlog::info("  E: 切换边界搜索模式 (0:原始算法, 1:改进算法-多候选边界)");
    spdlog::info("  SPACE: 暂停/继续");
    spdlog::info("  N: 单帧前进(暂停时)");
    spdlog::info("  Q: 退出");

    // 创建 HSV 调试窗口和滑条
    cv::namedWindow("HSV Tuning", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("H Min", "HSV Tuning", &hMin, 180);
    cv::createTrackbar("H Max", "HSV Tuning", &hMax, 180);
    cv::createTrackbar("V Min", "HSV Tuning", &sMin, 255);
    cv::createTrackbar("S Max", "HSV Tuning", &sMax, 255);
    cv::createTrackbar("V Min", "HSV Tuning", &vMin, 255);
    cv::createTrackbar("V Max", "HSV Tuning", &vMax, 255);

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
    bool isPaused = false;
    bool nextFrame = false;

    while (true)
    {
        if (isImageMode) {
            // 图片模式：重复使用同一张图片
            frame = staticImage.clone();
        } else {
            // 视频/摄像头模式：读取下一帧
            if (!isPaused || nextFrame) {
                if (!cap.read(frame)) {
                    spdlog::info("End of video stream");
                    break;
                }
                nextFrame = false;
            }
        }

        if (frame.empty())
        {
            continue;
        }

        // ========== 图像预处理 ==========
        cv::Mat banmachuli = frame.clone();
        cv::resize(banmachuli, banmachuli, cv::Size(), 0.5, 0.5);

        int yStart = frame.rows / 2 - 90 + 70;
        int roiHeight = static_cast<int>(frame.rows / 2.5);
        if (yStart < 0) yStart = 0;
        if (yStart + roiHeight > frame.rows) roiHeight = frame.rows - yStart;

        cv::Rect roiRect(0, yStart, frame.cols, roiHeight);
        cv::Mat cropped_image = frame(roiRect).clone();
        cv::resize(cropped_image, cropped_image, cv::Size(), 0.5, 0.5);

        cv::Mat hsv_image;
        cv::cvtColor(cropped_image, hsv_image, cv::COLOR_BGR2HSV);

        cv::Mat gray_image;
        cv::cvtColor(cropped_image, gray_image, cv::COLOR_BGR2GRAY);

        // 公共预处理：双边滤波+高斯滤波
        cv::Mat preprocessed_gray;
        cv::Mat blur_temp;
        cv::bilateralFilter(gray_image, blur_temp, 7, 60, 60);
        cv::GaussianBlur(blur_temp, preprocessed_gray, cv::Size(5, 5), 30);

        cv::Mat ca;
        cv::Mat binary_img;

        // 默认参数
        int houghMinLineLength = 25;
        int houghMaxLineGap = 5;
        double filterMinAngle1 = -90.0;
        double filterMaxAngle1 = -18.0;
        double filterMinAngle2 = 18.0;
        double filterMaxAngle2 = 90.0;
        cv::Size dilateKernelSize(2, 2);

        // 根据模式选择预处理方法
        if (preprocessMode == 0)   // 顶帽 + OTSU
        {
            cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(17, 17));
            cv::Mat tophat_img;
            cv::morphologyEx(preprocessed_gray, tophat_img, cv::MORPH_TOPHAT, element);
            cv::normalize(tophat_img, tophat_img, 0, 255, cv::NORM_MINMAX);
            cv::threshold(tophat_img, binary_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            cv::Canny(binary_img, ca, 30, 50);
        }
        else if (preprocessMode == 1)   // 自适应阈值
        {
            cv::adaptiveThreshold(preprocessed_gray, binary_img, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 21, -2);
            cv::medianBlur(binary_img, binary_img, 5);
            cv::Canny(binary_img, ca, 30, 50);
        }
        else if (preprocessMode == 2)   // Shadow 算法
        {
            cv::Mat blur_shadow;
            cv::bilateralFilter(gray_image, blur_shadow, 7, 75, 75);
            cv::GaussianBlur(blur_shadow, binary_img, cv::Size(5, 5), 1.5);
            cv::Canny(binary_img, ca, 40, 80);

            houghMinLineLength = 35;
            houghMaxLineGap = 10;
            filterMinAngle1 = -75.0;
            filterMaxAngle1 = -25.0;
            filterMinAngle2 = 25.0;
            filterMaxAngle2 = 75.0;
            dilateKernelSize = cv::Size(3, 3);
        }
        else if (preprocessMode == 3)   // HSV 阈值
        {
            cv::inRange(hsv_image, cv::Scalar(hMin, sMin, vMin), cv::Scalar(hMax, sMax, vMax), binary_img);
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            cv::morphologyEx(binary_img, binary_img, cv::MORPH_OPEN, kernel);
            cv::Canny(binary_img, ca, 30, 50);
        }

        // Canny边缘检测后的膨胀
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, dilateKernelSize);
        cv::Mat dilated_ca;
        cv::dilate(ca, dilated_ca, kernel, cv::Point(-1, -1), 1);

        // Hough直线检测
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(dilated_ca, lines, 1, CV_PI / 180, 70, houghMinLineLength, houghMaxLineGap);

        cv::Mat line_image = cv::Mat::zeros(dilated_ca.size(), CV_8UC1);
        std::vector<cv::Vec4i> filtered_lines;

        // 角度过滤（左边界）
        for (const auto &line : lines)
        {
            double angle_rad = std::atan2(line[3] - line[1], line[2] - line[0]);
            double angle_deg = angle_rad * 180.0 / CV_PI;
            if (angle_deg >= filterMinAngle1 && angle_deg <= filterMaxAngle1)
            {
                filtered_lines.push_back(line);
            }
        }
        for (const auto &line : filtered_lines)
        {
            cv::line(line_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255), 2, cv::LINE_AA);
        }

        // 角度过滤（右边界）
        filtered_lines.clear();
        for (const auto &line : lines)
        {
            double angle_rad = std::atan2(line[3] - line[1], line[2] - line[0]);
            double angle_deg = angle_rad * 180.0 / CV_PI;
            if (angle_deg >= filterMinAngle2 && angle_deg <= filterMaxAngle2)
            {
                filtered_lines.push_back(line);
            }
        }
        for (const auto &line : filtered_lines)
        {
            cv::line(line_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255), 2, cv::LINE_AA);
        }

        // 最终膨胀
        cv::Mat dilated_ca2;
        cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
        cv::dilate(line_image, dilated_ca2, kernel2, cv::Point(-1, -1), 1);

        // ========== 循迹边界检测 ==========
        Image_Handle(dilated_ca2);

        // ========== 可视化 ==========
        cv::Mat track_display = dilated_ca2.clone();
        Draw_Track_Result(track_display, dilated_ca2);

        cv::imshow("cropped", cropped_image);
        cv::imshow("binary", binary_img);
        cv::imshow("canny", ca);
        cv::imshow("hough_filtered", dilated_ca2);
        cv::imshow("track_result", track_display);

        // ========== 按键控制 ==========
        int delay = isPaused ? 0 : 1;
        int key = cv::waitKey(delay);

        if (key == 'q' || key == 'Q')
        {
            break;
        }
        else if (key == 'm' || key == 'M')
        {
            preprocessMode = (preprocessMode + 1) % 4;
            spdlog::info("切换预处理模式到: {}", preprocessMode);
        }
        else if (key == 'e' || key == 'E')
        {
            edgeSearchMode = (edgeSearchMode + 1) % 2;
            std::string mode_name = edgeSearchMode == 0 ? "原始算法" : "改进算法-多候选边界";
            spdlog::info("========================================");
            spdlog::info("切换边界搜索模式到: {} ({})", edgeSearchMode, mode_name);
            spdlog::info("========================================");

            // 在所有窗口上显示提示，持续1秒
            cv::Mat notification(100, 400, CV_8UC3, cv::Scalar(0, 0, 0));
            cv::putText(notification, "Edge Mode: " + mode_name, cv::Point(10, 50),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            cv::imshow("Mode Changed", notification);
            cv::waitKey(500);  // 显示0.5秒
            cv::destroyWindow("Mode Changed");
        }
        else if (key == ' ')
        {
            isPaused = !isPaused;
            spdlog::info("暂停: {}", isPaused);
        }
        else if (key == 'n' || key == 'N')
        {
            if (isPaused)
            {
                nextFrame = true;
            }
        }
    }

    cv::destroyAllWindows();
    return 0;
}
