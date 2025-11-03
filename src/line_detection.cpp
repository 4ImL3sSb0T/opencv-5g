#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <numeric>
#include <vector>
#include <cmath>
#include "garage.hpp"

#ifdef _WIN32
    // 设置控制台代码页为 UTF-8
#include <windows.h>
// 或者使用 system("chcp 65001 > nul");
#endif

// 定义常量
const int ROW = 96;
const int COL = 320;
const int setspeed1 = 100;
const int Bizhang_line_move = 60;
const int BZ_CACHE_FRAMES = 9;  // 障碍物检测多帧缓存
const int BZ_CONFIRM_FRAMES = 2;// 障碍物确认所需连续帧数
int erroe_xiuzheng = 0;

// 全局变量
int16_t Left_Line[ROW + 2], Right_Line[ROW + 2];
int16_t Mid_Line[ROW + 2];
int16_t Left_Add_Line[ROW + 2], Right_Add_Line[ROW + 2];
int16_t Left_Add_Flag[ROW + 2], Right_Add_Flag[ROW + 2];
int16_t Road_Width_Real[ROW + 2];
int16_t Road_Width_Add[ROW + 2];
int16_t Line_Count;
int16_t Out_Side = 0;

float Left_Ka = 0, Right_Ka = 0;
float Left_Kb = 1, Right_Kb = COL - 1;

int Interpolated_Liness[ROW + 2];

// 避障全局变量
int find_XYdata[3] = {0};                // 障碍物中心点坐标(x, y, 预留)
int BZ_con[BZ_CACHE_FRAMES] = {0};       // 障碍物检测结果缓存
int bizhangenable = 1;                   // 避障使能开关(1=允许, 0=禁止)
int BZ_Imageflag = 1;                    // 避障调试图像显示标志
cv::Mat cropped_imageddddd;              // 裁剪后原图（供障碍物检测用）
int BS_BZ_FLAG = 0;                      // 避障状态标志(0=无避障,1=有避障)

// 斑马线检测全局变量
int banmaxian_Y = 0;                     // 斑马线平均Y坐标
int banma111 = 0;                        // 斑马线调试显示标志1
int banma222 = 0;                        // 斑马线调试显示标志2

// 函数声明
int16_t Limit_Protect(int16_t num, int32_t min, int32_t max);
void Earge_Search_Mid(int16_t i, cv::Mat data, int16_t Mid, int16_t Left_Min, int16_t Right_Max, 
                     int16_t* Left_Line, int16_t* Right_Line, 
                     int16_t* Left_Add_Line, int16_t* Right_Add_Line, int mods);
int16_t First_Line_Handle(cv::Mat data);
void Mid_Line_Repair();
void LinearInterpolation();
int TUxiang_Init(cv::Mat data);
int Image_Handle22(cv::Mat data, cv::Mat YUANTU, cv::Mat BANMA);
int error_get(void);
// 车库入库误差（根据方向使用中线与对应边线的均值来牵引车辆贴近目标车库）
int garage_error_get(Garage::Direction dir);

// 斑马线检测函数
int BanMa_Find(cv::Mat BanMa_Find_data);

// 避障函数声明
int BZ_chuli(cv::Mat BZdata);               // 障碍物颜色检测
int BZ_PANDUAN_2(void);                     // 判断障碍物是否在赛道内
void bizhangBuxian(int data_X, int data_Y, int bizhang_fangxiang);  // 避障边界调整
void BZ_LuoJISET(void);                     // 避障逻辑控制
void BZ_Cache_Update(int detect_res);       // 障碍物检测结果缓存更新
int BZ_Cache_Check(void);                  // 多帧缓存结果校验

// 绝对值函数
int Q_jdz(int num) {
    return (num < 0) ? -num : num;
}

// 斑马线检测函数实现
int BanMa_Find(cv::Mat BanMa_Find_data) {
    // 定义感兴趣区域
    cv::Rect roi(100, 0, 120, 240);
    BanMa_Find_data = BanMa_Find_data(roi);
    // 压缩图像尺寸
    cv::resize(BanMa_Find_data, BanMa_Find_data, cv::Size(), 1, 0.5);
    // 将图像从 BGR 转换为灰度
    cv::Mat gray_image;
    cv::cvtColor(BanMa_Find_data, gray_image, cv::COLOR_BGR2GRAY);
    // 对灰度图进行双边滤波
    cv::Mat blur;
    cv::bilateralFilter(gray_image, blur, 7, 60, 60);
    // 对模糊后的图像进行高斯滤波
    cv::Mat gaussian_blur;
    cv::GaussianBlur(blur, gaussian_blur, cv::Size(5, 5), 30);
    // 使用 Canny 算子进行边缘检测
    cv::Mat ca;
    cv::Canny(gaussian_blur, ca, 30, 50);
    // 定义膨胀核
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    // 对边缘检测后的图像进行膨胀处理
    cv::Mat dilated_ca;
    cv::dilate(ca, dilated_ca, kernel, cv::Point(-1, -1), 1);
    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(dilated_ca, contours, cv::RetrievalModes::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // 创建一个新的图像来绘制轮廓
    cv::Mat contour_img = dilated_ca.clone();
    std::vector<int> Y_points;
    int count_BMX = 0;
    // 设置矩形宽高的最小和最大阈值
    const int min_wh = 5;
    const int max_wh = 55;
    // 遍历轮廓并筛选符合条件的矩形
    for (const auto& contour : contours) {
        cv::Rect rect = cv::boundingRect(contour);
        if (rect.height >= min_wh && rect.height < max_wh && rect.width >= min_wh && rect.width < max_wh) {
            if (rect.y >= 40 && rect.y <= 72) {
                // 调整矩形位置
                if (rect.y % 2 == 0) rect.y -= 1;
                if (rect.y >= 85 || (rect.x >= 20 && rect.x <= 300)) {
                    // 绘制矩形
                    cv::rectangle(contour_img, rect, cv::Scalar(255, 0, 0), 2);
                    Y_points.push_back(rect.y);
                    ++count_BMX;
                }
            }
        }
    }
    // 如果找到足够多的矩形，计算它们的平均位置
    if (count_BMX >= 4) {
        int sum_Y = std::accumulate(Y_points.begin(), Y_points.end(), 0);
        int average_Y = sum_Y / count_BMX;
        int pingJun = std::accumulate(Y_points.begin(), Y_points.end(), 0,
                                      [average_Y](int acc, int y) { return acc + std::abs(average_Y - y); }) / count_BMX;
        std::cout << "斑马线平均偏差: " << pingJun << std::endl;
        // 如果平均偏差小于阈值，返回成功
        if (pingJun < 5) {
            std::cout << "检测到斑马线！位置: " << average_Y << std::endl;
            return 1;
        }
    }

    // 显示轮廓图像
    if(banma222 == 1) {
        cv::imshow("斑马线检测", contour_img);
    }
    return 0;
}

// 原循迹函数实现
int16_t Limit_Protect(int16_t num, int32_t min, int32_t max) {
    if (num >= max)
        return max;
    else if (num <= min)
        return min;
    else
        return num;
}

void Earge_Search_Mid(int16_t i, cv::Mat data, int16_t Mid, int16_t Left_Min, int16_t Right_Max, 
                     int16_t* Left_Line, int16_t* Right_Line, 
                     int16_t* Left_Add_Line, int16_t* Right_Add_Line, int mods) {
    int16_t j;
    int16_t N = 6;

    Left_Add_Flag[i] = 1;
    Right_Add_Flag[i] = 1;

    Right_Line[i] = Right_Max;
    Left_Line[i] = Left_Min;

    // 左边线查找（从中线向左搜索黑白边界）
    for (j = Mid; j >= 10; j -= 4) {
        if ((data.at<uchar>(i, j) < 100) && (data.at<uchar>(i, j-4) < 100) && (data.at<uchar>(i, j-8) > 100)) {
            if (j >= 320/2 + 50) {
                j = j - 30;
            } else {
                Left_Add_Flag[i] = 0;
                Left_Line[i] = j;
                Left_Add_Line[i] = j;
                break;
            }
        }
    }

    // 右边线查找（从中线向右搜索黑白边界）
    for (j = Mid; j <= COL - 10; j += 4) {
        if ((data.at<uchar>(i, j) < 100) && (data.at<uchar>(i, j+4) < 100) && (data.at<uchar>(i, j+8) > 100)) {
            if (j <= 320/2 - 50) {
                j = j + 30;
            } else {
                Right_Add_Flag[i] = 0;
                Right_Line[i] = j;
                Right_Add_Line[i] = j;
                break;
            }
        }
    }

    // 左边线补线处理（未检测到边界时）
    if (Left_Add_Flag[i]) {
        if (i >= ROW - N)
            Left_Add_Line[i] = Right_Line[ROW - 1] - 200;
        else
            Left_Add_Line[i] = Right_Add_Line[ROW - 1] - 200;
    }

    // 右边线补线处理（未检测到边界时）
    if (Right_Add_Flag[i]) {
        if (i >= ROW - N)
            Right_Add_Line[i] = Left_Line[ROW - 1] + 200;
        else
            Right_Add_Line[i] = Left_Add_Line[i + 2] + 200;
    }

    Road_Width_Real[i] = Right_Line[i] - Left_Line[i];
    Road_Width_Add[i] = Right_Add_Line[i] - Left_Add_Line[i];
    Mid_Line[i] = (Right_Add_Line[i] + Left_Add_Line[i]) / 2;
}

int16_t First_Line_Handle(cv::Mat data) {
    Mid_Line[ROW + 1] = Mid_Line[ROW - 1];
    if (Mid_Line[ROW + 1] >= 320 - 40 || Mid_Line[ROW + 1] <= 40) {
        Mid_Line[ROW + 1] = COL / 2;
    }
    return 0;
}

void Mid_Line_Repair() {
    for (int i = ROW-1; i >= 9; i -= 2) {
        Mid_Line[i] = (Right_Add_Line[i] + Left_Add_Line[i]) / 2;
    }
}

void LinearInterpolation() {
    float data = 0;  // 保留原变量，不影响功能
    for (int i = ROW-1; i >= 9; i -= 2) {
        Interpolated_Liness[i] = (Mid_Line[i] + Mid_Line[i - 2]) / 2;
        Mid_Line[i - 2] = Interpolated_Liness[i];
    }
}

int error_get(void) {
    long error_in = 0;
    int error_out = 0;
    long Weight_Count = 0;
    int j = 0;
    
    uint8_t Weight_th[110] = {
        2, 2, 2, 2, 2, 3, 3, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
        4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
        3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
        7, 7, 7, 7, 7, 7, 8, 8, 8, 9,
        8, 8, 8, 8, 8, 8, 8, 8, 8, 9,
        5, 5, 5, 5, 5, 5, 6, 6, 6, 6,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };
    
    for (int i = ROW - 1; i >= 9; i -= 2) {
        error_in += (Mid_Line[i] - 160) * Weight_th[j];
        Weight_Count += Weight_th[j];
        j++;
    }
    
    error_out = error_in / Weight_Count;
    
    if (error_out < -160) error_out = -160;
    if (error_out > 160) error_out = 160;
    
    return error_out/4;
}

// 车库入库误差计算
int garage_error_get(Garage::Direction dir) {
    long error_in = 0;
    int error_out = 0;
    long Weight_Count = 0;
    int j = 0;

    uint8_t Weight_th[110] = {
        2, 2, 2, 2, 2, 3, 3, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
        4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
        3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
        7, 7, 7, 7, 7, 7, 8, 8, 8, 9,
        8, 8, 8, 8, 8, 8, 8, 8, 8, 9,
        5, 5, 5, 5, 5, 5, 6, 6, 6, 6,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };

    for (int i = ROW - 1; i >= 9; i -= 2) {
        int16_t side_line = (dir == Garage::LEFT) ? Left_Add_Line[i] : Right_Add_Line[i];
        // 目标线：中线与目标边线的均值，避免过度贴边且能引导入库
        int16_t target_line = (Mid_Line[i] + side_line) / 2;
        error_in += (target_line - 160) * Weight_th[j];
        Weight_Count += Weight_th[j];
        j++;
    }

    if (Weight_Count == 0) return 0;
    error_out = error_in / Weight_Count;

    if (error_out < -160) error_out = -160;
    if (error_out > 160) error_out = 160;

    return error_out / 4;
}

// 避障函数实现
// 1. 障碍物检测结果缓存更新
void BZ_Cache_Update(int detect_res) {
    for (int i = 0; i < BZ_CACHE_FRAMES - 1; i++) {
        BZ_con[i] = BZ_con[i + 1];
    }
    BZ_con[BZ_CACHE_FRAMES - 1] = detect_res;
}

// 2. 多帧缓存结果校验
int BZ_Cache_Check(void) {
    int valid_count = 0;
    for (int i = 0; i < BZ_CACHE_FRAMES; i++) {
        if (BZ_con[i] == 1) valid_count++;
    }
    return (valid_count >= BZ_CONFIRM_FRAMES) ? 1 : 0;
}

// 3. 障碍物颜色检测（优化HSV阈值）
int BZ_chuli(cv::Mat BZdata) {
    cv::Mat hsv_img;
    cv::cvtColor(cropped_imageddddd, hsv_img, cv::COLOR_BGR2HSV);

    // 优化的蓝色障碍物HSV阈值（适应更多光照条件）
    cv::Scalar blue_lower(136, 150, 49);
    cv::Scalar blue_upper(179, 255, 255);
    cv::Mat blue_mask;
    cv::inRange(hsv_img, blue_lower, blue_upper, blue_mask);

    // 形态学操作：先腐蚀去小噪声，再膨胀增强色块
    cv::Mat kernel_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::erode(blue_mask, blue_mask, kernel_erode, cv::Point(-1, -1), 1);
    cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(blue_mask, blue_mask, kernel_dilate, cv::Point(-1, -1), 1);

    // 显示障碍物检测掩码（调试用）
    if (BZ_Imageflag == 1) {
        cv::imshow("Obstacle Mask (Blue)", blue_mask);
    }

    // 查找最大色块轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double max_area = 0;
    int max_area_idx = -1;

    for (int i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_area_idx = i;
        }
    }

    // 有效障碍物判定（面积>5像素）
    if (max_area_idx >= 0 && max_area > 50) {
        cv::Rect bound_rect = cv::boundingRect(contours[max_area_idx]);
        // 坐标缩放（精确匹配赛道图像分辨率）
        find_XYdata[0] = (bound_rect.x + bound_rect.width / 2) * (COL / (float)cropped_imageddddd.cols);
        find_XYdata[1] = (bound_rect.y + bound_rect.height / 2) * (ROW / (float)cropped_imageddddd.rows);
        
        // 在裁剪图上标记障碍物
        cv::rectangle(cropped_imageddddd, bound_rect, cv::Scalar(0, 0, 255), 2);
        cv::circle(cropped_imageddddd, 
                  cv::Point(bound_rect.x + bound_rect.width/2, bound_rect.y + bound_rect.height/2), 
                  3, cv::Scalar(0, 255, 0), -1);
        
        // 显示标记后的裁剪图
        if (BZ_Imageflag == 1) {
            cv::imshow("Cropped Image (Obstacle Marked)", cropped_imageddddd);
        }
        return 1;  // 检测到障碍物
    }

    return 0;  // 未检测到障碍物
}

// 4. 判断障碍物是否在赛道内
int BZ_PANDUAN_2(void) {
    int target_row = 0;
    // 匹配障碍物所在赛道行（允许±2行误差）
    for (int i = ROW-1; i >= 9; i -= 2) {
        if (find_XYdata[1] >= i - 2 && find_XYdata[1] <= i + 2) {
            target_row = i;
            break;
        }
    }

    // 输出调试信息
    std::cout << "[Debug] Obstacle Position: (" << find_XYdata[0] << "," << find_XYdata[1] 
              << "), Target Row: " << target_row << std::endl;

    // 无效行或远距离物体判定为赛道外
    if (target_row == 0 || find_XYdata[1] <= 10) {
        BS_BZ_FLAG = 0;
        return 0;
    }

    // 核心判定：横坐标在赛道左右边界内
    if (find_XYdata[0] >= Left_Add_Line[target_row] - 15 && 
        find_XYdata[0] <= Right_Add_Line[target_row] + 15) {
        BS_BZ_FLAG = 1;
        return 1;  // 赛道内障碍物，需避障
    } else {
        BS_BZ_FLAG = 0;
        return 0;  // 赛道外障碍物，无需避障
    }
}

// 5. 避障边界调整（修正方向逻辑）
void bizhangBuxian(int data_X, int data_Y, int bizhang_fangxiang) {
    const int avoid_offset = Bizhang_line_move;  // 避障偏移量
    // 只调整前方30行赛道
    for (int i = ROW-1; i >= ROW - 60; i -= 2) {
        if (bizhang_fangxiang == 0) {  // 向左避障：调整左边界
            Left_Add_Line[i] = Limit_Protect(data_X - avoid_offset, 
                                            10,                      // 不超出图像左边界
                                            Right_Add_Line[i] - 20); // 避免与右边界重叠
        } else if (bizhang_fangxiang == 1) {  // 向右避障：调整右边界
            Right_Add_Line[i] = Limit_Protect(data_X + avoid_offset, 
                                             Left_Add_Line[i] + 20,   // 避免与左边界重叠
                                             COL - 10);              // 不超出图像右边界
        }
    }
}

// 6. 避障逻辑控制（修正状态机）
void BZ_LuoJISET(void) {
    static int obstacle_track = 0;  // 障碍物跟踪状态：0=未跟踪，1=跟踪中，2=跟踪完成
    static int avoid_dir = 0;       // 避障方向：0=左，1=右（交替切换）
    static int frame_cnt = 0;       // 状态确认计数器

    // 状态1：障碍物从远处进入视野（y>ROW-20）
    if (find_XYdata[1] > ROW - 20 && obstacle_track == 0) {
        frame_cnt++;
        if (frame_cnt >= 1) {  // 3帧确认
            obstacle_track = 1;
            frame_cnt = 0;
            std::cout << "[Obstacle Track] Start tracking" << std::endl;
        }
    }

    // 状态2：障碍物接近（y≤ROW-20），执行避障
    if (obstacle_track == 1 && find_XYdata[1] <= ROW - 20) {
        bizhangBuxian(find_XYdata[0], find_XYdata[1], avoid_dir);
        std::cout << "[Avoid Action] Direction: " << (avoid_dir ? "Right" : "Left") << std::endl;
        frame_cnt++;
        if (frame_cnt >= 5) {  // 避障5帧后切换状态
            obstacle_track = 2;
            frame_cnt = 0;
        }
    }

    // 状态3：障碍物远离（y≤10），切换避障方向
    if (obstacle_track == 2 && find_XYdata[1] <= 10) {
        avoid_dir = (avoid_dir == 0) ? 1 : 0;  // 交替切换方向
        obstacle_track = 0;
        std::cout << "[Track End] Switch next avoid direction to " << (avoid_dir ? "Right" : "Left") << std::endl;
    }
}

// 修改TUxiang_Init：完善避障流程
int TUxiang_Init(cv::Mat data) {
    cv::Rect roi_rect(0, (data.rows / 2 - 90 + 70), data.cols, (data.rows / 2.5));
    cv::Mat cropped_image = data(roi_rect);
    // 保存裁剪图到全局变量
    cropped_imageddddd = cropped_image.clone();
    
    // 图像缩放
    if (!cropped_image.empty()) {
        cv::resize(cropped_image, cropped_image, cv::Size(), 0.5, 0.5);
    } else {
        std::cerr << "[Image Error] Cropped image is empty" << std::endl;
        return 0;
    }

    // 原循迹图像处理流程
    cv::Mat hsv_image;
    cv::cvtColor(cropped_image, hsv_image, cv::COLOR_BGR2HSV);
    
    cv::Mat gray_image;
    cv::cvtColor(cropped_image, gray_image, cv::COLOR_BGR2GRAY);
    
    cv::Mat blur;
    cv::bilateralFilter(gray_image, blur, 7, 60, 60);
    
    cv::Mat gaussian_blur;
    cv::GaussianBlur(blur, gaussian_blur, cv::Size(5, 5), 30);
    
    cv::Mat ca;
    cv::Canny(gaussian_blur, ca, 30, 50);
    
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::Mat dilated_ca;
    cv::dilate(ca, dilated_ca, kernel, cv::Point(-1, -1), 1);
    
    // 霍夫线段检测与过滤
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(dilated_ca, lines, 1, CV_PI / 180, 70, 25, 5);
    
    cv::Mat line_image = cv::Mat::zeros(dilated_ca.size(), CV_8UC1);
    std::vector<cv::Vec4i> filtered_lines;
    
    // 过滤左边界线段（角度-90°~-18°）
    double min_angle = -90;
    double max_angle = -18;
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i line = lines[i];
        double angle_rad = atan2(line[3] - line[1], line[2] - line[0]);
        double angle_deg = angle_rad * 180 / CV_PI;
        if (angle_deg >= min_angle && angle_deg <= max_angle) {
            filtered_lines.push_back(line);
        }
    }
    for (size_t i = 0; i < filtered_lines.size(); i++) {
        cv::line(line_image, cv::Point(filtered_lines[i][0], filtered_lines[i][1]),
                cv::Point(filtered_lines[i][2], filtered_lines[i][3]), cv::Scalar(255), 2, cv::LINE_AA);
    }
    
    // 过滤右边界线段（角度18°~90°）
    min_angle = 18;
    max_angle = 90;
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i line = lines[i];
        double angle_rad = atan2(line[3] - line[1], line[2] - line[0]);
        double angle_deg = angle_rad * 180 / CV_PI;
        if (angle_deg >= min_angle && angle_deg <= max_angle) {
            filtered_lines.push_back(line);
        }
    }
    for (size_t i = 0; i < filtered_lines.size(); i++) {
        cv::line(line_image, cv::Point(filtered_lines[i][0], filtered_lines[i][1]),
                cv::Point(filtered_lines[i][2], filtered_lines[i][3]), cv::Scalar(255), 2, cv::LINE_AA);
    }
    
    // 线段图像膨胀
    cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::Mat dilated_ca2;
    cv::dilate(line_image, dilated_ca2, kernel2, cv::Point(-1, -1), 1);
    
    // 原循迹核心：边界检测与中线计算
    int car_error = Image_Handle22(dilated_ca2, hsv_image, dilated_ca);

    // 避障流程：仅在避障使能时执行
    if (bizhangenable == 1) {
        int bz_detect = BZ_chuli(hsv_image);  // 1. 障碍物检测
        BZ_Cache_Update(bz_detect);           // 2. 更新检测缓存
        int bz_confirm = BZ_Cache_Check();    // 3. 多帧确认障碍物
        
        if (bz_confirm == 1) {
            int bz_in_road = BZ_PANDUAN_2();  // 4. 判断是否在赛道内
            if (bz_in_road == 1) {
                BZ_LuoJISET();                // 5. 执行避障逻辑
                Mid_Line_Repair();            // 6. 重新计算避障后的中线
                car_error = error_get();      // 7. 基于新中线更新转向误差
                std::cout << "[Avoid Update] New car_error: " << car_error << std::endl;
            }
        }
    }

    // 车库模式：根据 Garage::Direction 切换误差（忽略中间黄色胶带，仅用扫线结果）
    if (Garage::currentDirection == Garage::LEFT || Garage::currentDirection == Garage::RIGHT) {
        car_error = garage_error_get(Garage::currentDirection);
    }

    return car_error;
}

// 原Image_Handle22函数
int Image_Handle22(cv::Mat data, cv::Mat YUANTU, cv::Mat BANMA) {
    int16_t i;
    Line_Count = 0;
    
    // 初始化补线标志
    for (i = ROW-1; i >= 9; i -= 2) {
        Left_Add_Flag[i] = 1;
        Right_Add_Flag[i] = 1;
    }
    
    // 首行处理
    int y = First_Line_Handle(data);
    
    // 逐行检测赛道边界
    for (i = ROW-1; i >= 9; i -= 2) {
        Line_Count = i;
        Earge_Search_Mid(i, data, Mid_Line[i + 2], 1, COL - 1, Left_Line, Right_Line, 
                         Left_Add_Line, Right_Add_Line, 0);
    }
    
    // 中线插值与修复
    LinearInterpolation();
    Mid_Line_Repair();
    
    // 计算转向误差
    int errroer_car = error_get();
    return (errroer_car + erroe_xiuzheng);
}

// 主程序
int main() {

#ifdef _WIN32
    // 设置控制台代码页为 UTF-8
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    // 或者使用 system("chcp 65001 > nul");
#endif
    // 打开摄像头
    cv::VideoCapture cap("../img/camera_record1.mp4");
    if (!cap.isOpened()) {
        std::cerr << "无法打开摄像头！" << std::endl;
        return -1;
    }
    
    // 设置摄像头分辨率
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    
    // 主循环
    while (true) {
        cv::Mat frame;
        cap >> frame;  // 获取一帧图像
        if (frame.empty()) {
            std::cerr << "无法获取图像帧！" << std::endl;
            break;
        }
        
        // 斑马线检测
        int zebra_result = BanMa_Find(frame.clone());
        if (zebra_result == 1) {
            std::cout << "********** 检测到斑马线，准备停车！ **********" << std::endl;
            // 这里可以添加停车逻辑
            // 例如：设置速度为零，然后退出循环
            
        }
        
        // 图像处理与循迹
        int error = TUxiang_Init(frame);
        std::cout << "当前转向误差: " << error << std::endl;
        
        // 显示原始图像
        cv::imshow("原始图像", frame);
        
        // 按键检测（ESC退出）
        if (cv::waitKey(1) == 27) {
            break;
        }
    }
    
    // 释放资源
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
