#include "image_Q.hpp"
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <string>

// 全局变量定义
int car_speed = setspeed1; // 车辆速度，初始值来自serial.h
int banmaxian_Y = 0;       // 检测到的斑马线Y坐标
int16 EdgeThres = 18;      // 图像处理中的跳变沿阈值
float BlackThres = 80;     // 黑白二值化阈值
cv::Mat Imagel;            // 存储黑白图像或其他中间处理图像
int16 Left_Line[ROW + 2],
    Right_Line[ROW + 2]; // 存储每行检测到的实际左右边界的横坐标
int16 Mid_Line[ROW + 2]; // 存储每行计算出的赛道中线的横坐标
int16 Left_Add_Line[ROW + 2],
    Right_Add_Line[ROW + 2]; // 存储经过补线处理后的左右边界横坐标
int16 Left_Add_Flag[ROW + 2],
    Right_Add_Flag[ROW + 2];           // 标记每行是否需要进行左右边界补线
int16 Road_Width_Real[ROW + 2];        // 存储每行检测到的实际赛道宽度
int16 Road_Width_Add[ROW + 2];         // 存储补线后的赛道宽度
int16 island_flag = 0;                 // 环岛标志: 0-非环岛, 1-左环岛, 2-右环岛
int16 Road_Width_Min;                  // 最小赛道宽度
int16 Left_Add_Start, Right_Add_Start; // 左右补线的起始行号
int16 Left_Add_Stop, Right_Add_Stop;   // 左右补线的结束行号
int16 Line_Count;                      // 记录成功识别到赛道边界的行数
int16 Out_Side = 0;                    // 丢线控制标志
float Left_Ka = 0, Right_Ka = 0;       // 最小二乘法拟合直线的斜率
float Left_Kb = 1, Right_Kb = COL - 1; // 最小二乘法拟合直线的截距

int BS_BZ_FLAG = 0;    // 避障减速标志
int CAR_STOP_FLAG = 0; // 停车标志
int TIME_BIZHANG = 1;  // 避障功能启用标志，到时间后开启

//-------------------------------------------------------------------------------------
int BMGet = 0;        // 斑马线检测结果标志
int BZ_FLAG_FLAG = 0; // 避障相关标志
int banmaenable = 1;  // 斑马线检测功能使能标志: 1-允许, 2-不允许
//-------------------------------------------------------------------------------------
int CAR_FaChe(cv::Mat FaChe_data);          // 发车函数声明
int BanMa_Find111(cv::Mat BanMa_Find_data); // 斑马线检测函数声明
//-------------------------------------------------------------------------------------
// UI调试相关标志
int red_set = 0;    // 红色阈值设置标志
int yellow_set = 0; // 黄色阈值设置标志
int saidao = 0;     // 赛道图像显示标志1
int saidao1 = 0;    // 赛道图像显示标志2
int banma111 = 0;   // 斑马线检测图像显示标志1
int banma222 = 0;   // 斑马线检测图像显示标志2

/*-------------------------------------------------------------------------------------------------------------------
 * @brief   限幅保护函数
 * @param   num   需要限制的数值
 * @param   min   允许的最小值
 * @param   max   允许的最大值
 * @return  int16 限制在[min, max]范围内的值
 * @note    用于防止补线等计算出的坐标越出图像边界
-------------------------------------------------------------------------------------------------------------------*/
int16 Limit_Protect(int16 num, int32 min, int32 max) {
  if (num >= max)
    return max;
  else if (num <= min)
    return min;
  else
    return num;
}
/*-------------------------------------------------------------------------------------------------------------------
 * @brief   根据直线方程计算点的坐标
 * @param   i     输入的行号 (相当于x)
 * @param   Ka    直线的斜率
 * @param   Kb    直线的截距
 * @return  int16 计算出的横坐标 (相当于y)，并经过限幅保护
 * @note    公式为: y = Ka * x + Kb
-------------------------------------------------------------------------------------------------------------------*/
int16 Fit_Point(uint8 i, float Ka, float Kb) {
  float res;
  int16 Result;
  res = i * Ka + Kb;
  Result = Limit_Protect((int16)res, 1, COL - 1);
  return Result;
}
/*-------------------------------------------------------------------------------------------------------------------
 * @brief   计算两个无符号字节的差的绝对值
 * @param   Data     当前值
 * @param   Set_num  目标值
 * @return  char     差的绝对值
-------------------------------------------------------------------------------------------------------------------*/
char Error_Transform(uint8 Data, uint8 Set_num) {
  char Error;

  Error = Set_num - Data;
  if (Error < 0) {
    Error = -Error;
  }

  return Error;
}

/**
 * @brief 计算一个整数的绝对值
 * @param A 输入的整数
 * @return int A的绝对值
 */
int Q_jdz(int A) {
  if (A < 0)
    A = -A;

  return A;
}

/*-------------------------------------------------------------------------------------------------------------------
 * @brief   曲线拟合函数1 (两点法)
 * @param   Ka       指向斜率的指针 (输出)
 * @param   Kb       指向截距的指针 (输出)
 * @param   Start    指向起始行号的指针 (输入/输出)，函数会寻找突出点并更新此值
 * @param   Line_Add 边界线坐标数组
 * @param   Mode     模式: 1-左边界, 2-右边界
 * @note    通过寻找一个突出点和其后一点来确定一条直线，用于补线。
-------------------------------------------------------------------------------------------------------------------*/
void Curve1_Fitting(float *Ka, float *Kb, int16 *Start, int16 *Line_Add,
                    int16 Mode) {
  int i;      // 用于内部循环
  int _start; // 临时变量储存起始行
  *Start += 2; // i行已经需要补线，肯定找前一行数据，此行扫描到边界
  _start = *Start;
  if (Mode == 2) // 右补线
  {
    for (i = _start; i <= _start + 6; i += 2) // 寻找右边界近处突出点
    {
      if (Right_Line[i] < Right_Line[*Start])
        *Start = i; // 更新突出点
    }
    if (*Start >= 59)
      *Start = 57;

    *Ka = 1.0 * (Line_Add[*Start + 2] - Line_Add[*Start]) / 2; // 计算Ka
    if (*Ka < 0) // 防止出现负值
      *Ka = 0;
  } else if (Mode == 1) {
    for (i = _start; i <= _start + 6; i += 2) // 寻找左边界近处突出点
      if (Left_Line[i] > Left_Line[*Start])
        *Start = i; // 更新突出点
    if (*Start >= 59)
      *Start = 57;
    *Ka = 1.0 * (Line_Add[*Start + 2] - Line_Add[*Start]) / 2; // 计算Ka
    if (*Ka > 0) // 防止出现负值
      *Ka = 0;
  }
  *Kb = 1.0 * Line_Add[*Start] - (*Ka * (*Start)); // 代入公式计算Kb
}
/*-------------------------------------------------------------------------------------------------------------------
 * @brief   曲线拟合函数2 (带偏移的两点法)
 * @param   Ka    指向斜率的指针 (输出)
 * @param   Kb    指向截距的指针 (输出)
 * @param   Start 起始行号
 * @param   End   结束行号
 * @param   Line  边界线坐标数组
 * @param   Mode  模式: 1-左边界, 2-右边界
 * @param   num   偏移量
 * @note    在两点法基础上增加一个偏移量，用于特殊情况下的补线。
-------------------------------------------------------------------------------------------------------------------*/
void Curve2_Fitting(float *Ka, float *Kb, uint8 Start, uint8 End, int16 *Line,
                    int16 Mode, int16 num) {
  if (Mode == 1) {
    *Ka = 1.0 * ((Line[Start] + num) - Line[End]) / (Start - End);
    *Kb = 1.0 * Line[End] - (*Ka * End);
  } else {
    *Ka = 1.0 * ((Line[Start] - num) - Line[End]) / (Start - End);
    *Kb = 1.0 * Line[End] - (*Ka * End);
  }
}
/*-------------------------------------------------------------------------------------------------------------------
 * @brief   曲线拟合函数3 (标准两点法)
 * @param   Ka    指向斜率的指针 (输出)
 * @param   Kb    指向截距的指针 (输出)
 * @param   Start 起始行号
 * @param   End   结束行号
 * @param   Line  边界线坐标数组
 * @param   Mode  模式: 1-左边界, 2-右边界
 * @note    环岛检测专用，最常规的两点法求直线方程。
-------------------------------------------------------------------------------------------------------------------*/
void Curve3_Fitting(float *Ka, float *Kb, uint8 Start, uint8 End, int16 *Line,
                    int16 Mode) // 环岛检测专用，最正规的求法
{
  if (Mode == 1) {
    *Ka = 1.0 * ((Line[Start]) - Line[End]) / (Start - End);
    *Kb = 1.0 * Line[End] - (*Ka * End);
  } else {
    *Ka = 1.0 * ((Line[Start]) - Line[End]) / (Start - End);
    *Kb = 1.0 * Line[End] - (*Ka * End);
  }
}

/*-------------------------------------------------------------------------------------------------------------------
 * @brief   边界搜索与处理函数
 * @param   i              当前处理的行号
 * @param   data           输入的二值化图像
 * @param   Mid            上一行的中线横坐标，作为本行搜索的起始点
 * @param   Left_Min       左边界搜索的最小横坐标
 * @param   Right_Max      右边界搜索的最大横坐标
 * @param   Left_Line      存储找到的左边界 (输出)
 * @param   Right_Line     存储找到的右边界 (输出)
 * @param   Left_Add_Line  存储补线后的左边界 (输出)
 * @param   Right_Add_Line 存储补线后的右边界 (输出)
 * @param   mods           模式参数 (未使用)
 * @note 从上一行中点向两侧搜索黑白跳变点作为边界。如果未找到，则进行补线处理。
-------------------------------------------------------------------------------------------------------------------*/
void Earge_Search_Mid(int16 i, cv::Mat data, int16 Mid, int16 Left_Min,
                      int16 Right_Max, int16 *Left_Line, int16 *Right_Line,
                      int16 *Left_Add_Line, int16 *Right_Add_Line, int mods) {
  int16 j; // 用于内部列循环
  int16 N =
      6; /*前N行丢线特殊处理，近处丢线如果不特殊处理，补线过于偏的话，因为所占权重大，会导致的影响比较大*/

  Left_Add_Flag[i] = 1; // 初始化补线标志位为1 (需要补线)
  Right_Add_Flag[i] = 1;

  Right_Line[i] = Right_Max; // 给定边界初始值，一般为1和COL-1
  Left_Line[i] = Left_Min;
  int16 avg_width = 0;
  int valid_width_count = 0;
  // 计算历史有效赛道宽度的平均值
  for (int k = ROW - 1; k >= 9; k -= 2) {
    if (Road_Width_Real[k] > 50 && Road_Width_Real[k] < 300) { // 过滤异常值
      avg_width += Road_Width_Real[k];
      valid_width_count++;
    }
  }
  if (valid_width_count > 0)
    avg_width /= valid_width_count;
  else
    avg_width = 150; // 如果没有有效历史数据，使用默认宽度

  /*左边线查找*/
  for (j = Mid; j >= 10; j -= 4) // 以前一行中点为起点向左查找边界
  {
    if ((data.at<uchar>(i, j) < 100) && (data.at<uchar>(i, j - 4) < 100) &&
        (data.at<uchar>(i, j - 8) > 100)) /*黑白跳变判断*/
    {
      if (j >= 320 / 2 + 50) // 如果在图像右半边找到了左边界，可能是干扰
      {
        j = j - 30; // 向左跳过一段距离继续找
      } else {
        Left_Add_Flag[i] = 0; // 左边界不需要补线，清除标志位
        Left_Line[i] = j;     // 记录当前j值为本行实际左边界
        Left_Add_Line[i] = j; // 记录实际左边界为补线左边界
        break;
      }
    }
  }
  /*右边线查找*/
  for (j = Mid; j <= COL - 10; j += 4) // 以前一行中点为起点向右查找右边界
  {
    if ((data.at<uchar>(i, j) < 100) && (data.at<uchar>(i, j + 4) < 100) &&
        (data.at<uchar>(i, j + 8) > 100)) {
      if (j <= 320 / 2 - 50) // 如果在图像左半边找到了右边界，可能是干扰
      {
        j = j + 30; // 向右跳过一段距离继续找
      } else {
        Right_Add_Flag[i] = 0; // 右边界不需要补线，清除标志位
        Right_Line[i] = j;     // 记录当前j值为本行右边界
        Right_Add_Line[i] = j; // 记录实际右边界为补线右边界
        break;
      }
    }
  }

  /*左边线补线处理，注意只更新Left_Add_Line数组，不更新Left_Line数组*/
  if (Left_Add_Flag[i]) // 为1表示：没找到左边界需要补线
  {
    if (i >= ROW - N) // 近处丢线处理
      Left_Add_Line[i] =
          Right_Line[ROW - 1] - 200; // 使用底行右边界减去一个固定宽度
    else                             // 远处丢线处理
      Left_Add_Line[i] = Right_Add_Line[ROW - 1] - 200; // 使用前一行的补线数据
  }
  /*右边线补线处理，注意只更新Right_Add_Line数组，不更新Right_Line数组*/
  if (Right_Add_Flag[i]) // 为1表示：没找到右边界需要补线
  {
    if (i >= ROW - N) // 近处丢线处理
    {
      Right_Add_Line[i] =
          Left_Line[ROW - 1] + 200; // 使用底行左边界加上一个固定宽度
    } else                          // 远处丢线处理
    {
      Right_Add_Line[i] = Left_Add_Line[i + 2] + 200; // 使用前一行的补线数据
    }
  }

  Road_Width_Real[i] = Right_Line[i] - Left_Line[i]; // 计算实际赛道宽度
  Road_Width_Add[i] = Right_Add_Line[i] - Left_Add_Line[i]; // 计算补线赛道宽度
  Mid_Line[i] = (Right_Add_Line[i] + Left_Add_Line[i]) / 2; // 计算本行中线
}
//=====================================================================================================================================
/**
 * @brief 首行处理函数
 * @param data 输入的图像数据 (未使用)
 * @return int 总是返回0
 * @note  为从下往上扫线的第一行（虚拟行）提供一个中点初始值。
 *        这个中点是根据图像最底部的中线来确定的，并做了防呆处理。
 */
int16 First_Line_Handle(cv::Mat data) {
  static long mind_jundata[5];
  Mid_Line[ROW + 1] =
      Mid_Line[ROW - 1]; // 使用倒数第一行的中线作为虚拟首行的中线
  if (Mid_Line[ROW + 1] >= 320 - 40 ||
      Mid_Line[ROW + 1] <= 40) // 如果中线过于靠边
  {
    Mid_Line[ROW + 1] = COL / 2; // 则强制复位到图像中心
  }
  return 0; // 返还0 表示成功
}
/*-------------------------------------------------------------------------------------------------------------------
 * @brief   中线修复函数
 * @note
在所有行的边界都经过查找和补线处理后，重新计算一遍所有行的中线，确保中线是基于最终的边界数据。
-------------------------------------------------------------------------------------------------------------------*/
void Mid_Line_Repair() // 中线修复
{
  for (int i = ROW - 1; i >= 9; i -= 2) // 从图像底部向上遍历
  {
    Mid_Line[i] =
        (Right_Add_Line[i] + Left_Add_Line[i]) / 2; // 中线 = (左边界+右边界)/2
  }
}
int Interpolated_Liness[ROW + 2]; //  存储插值平滑后的中线数组
/**
 * @brief 对中线进行线性插值平滑
 * @note
 * 遍历中线数组，将当前点和前一个点（更远处的点）的平均值作为新的前一个点的值，
 *        以此来平滑中线，减少突变。
 */
void LinearInterpolation(void) {
  float data = 0;
  // 线性插值
  for (int i = ROW - 1; i >= 9; i -= 2) {
    Interpolated_Liness[i] = (Mid_Line[i] + Mid_Line[i - 2]) / 2;
    Mid_Line[i - 2] = Interpolated_Liness[i];
  }
}
// 创建视频写入对象，用于录制处理过程
cv::VideoWriter writer("output288.avi",
                       cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 60,
                       cv::Size(320, 96));
cv::Mat cropped_imageddddd; // 存储裁剪后的图像，供其他函数使用
cv::Mat banmachuli;         // 存储用于斑马线处理的图像

/**
 * @brief 计算数组中有效坐标的数量
 * @param line_array 坐标数组
 * @param total_rows 数组总行数
 * @return int 有效坐标的数量
 */
int count_valid_lines(int line_array[], int total_rows) {
  int valid_count = 0;
  for (int i = 0; i < total_rows; i++) {
    // 假设“无效坐标”是0或超出图像宽度
    if (line_array[i] > 0 && line_array[i] < 320) { // 320是示例宽度
      valid_count++;
    }
  }
  return valid_count;
}
int right_valid_count; // 右边界有效点计数
int left_valid_count;  // 左边界有效点计数

/**
 * @brief 图像预处理总函数
 * @param data 输入的原始摄像头图像 (BGR格式)
 * @return int 计算出的车辆循迹误差值
 * @note 这是图像处理的入口函数，完成从图像采集到误差计算的全过程。
 */
int TUxiang_Init(cv::Mat data) // 图像预处理
{
  // 1. 图像裁剪与缩放
  banmachuli = data; // 备份原始图像给斑马线处理
  cv::resize(banmachuli, banmachuli, cv::Size(), 0.5, 0.5); // 缩放备份图像
  cv::Rect roi_rect(0, (data.rows / 2 - 90 + 70), data.cols,
                    (data.rows / 2.5));   // 定义感兴趣区域(ROI)，提取赛道部分
  cv::Mat cropped_image = data(roi_rect); // 裁剪图像
  cropped_imageddddd = cropped_image;     // 全局备份裁剪后的图像
  if (!cropped_image.empty()) {
    cv::resize(cropped_image, cropped_image, cv::Size(), 0.5,
               0.5); // 缩放裁剪后的图像以提高处理速度
  } else {
    std::cout << "Cropped image is empty!" << std::endl;
    return 0; // 如果裁剪失败则返回
  }
  writer.write(cropped_image); // 将当前帧写入视频文件

  // 2. 颜色空间转换与滤波
  cv::Mat hsv_image;
  cv::cvtColor(cropped_image, hsv_image,
               cv::COLOR_BGR2HSV); // BGR -> HSV，用于颜色识别
  cv::Mat gray_image;
  cv::cvtColor(cropped_image, gray_image, cv::COLOR_BGR2GRAY); // BGR -> 灰度图
  cv::Mat blur;
  cv::bilateralFilter(gray_image, blur, 7, 60, 60); // 双边滤波，保边去噪
  cv::Mat gaussian_blur;
  cv::GaussianBlur(blur, gaussian_blur, cv::Size(5, 5),
                   30); // 高斯滤波，进一步平滑

  // 3. 边缘与直线检测
  cv::Mat ca;
  cv::Canny(gaussian_blur, ca, 30, 50); // Canny边缘检测
  cv::Mat kernel =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)); // 定义膨胀核
  cv::Mat dilated_ca;
  cv::dilate(ca, dilated_ca, kernel, cv::Point(-1, -1),
             1); // 膨胀操作，连接断开的边缘
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(dilated_ca, lines, 1, CV_PI / 180, 70, 25,
                  5); // 霍夫概率直线检测

  // 4. 直线筛选与绘制
  cv::Mat line_image = cv::Mat::zeros(
      dilated_ca.size(), CV_8UC1); // 创建一个黑色背景图用于绘制筛选后的直线
  std::vector<cv::Vec4i> filtered_lines;
  // 筛选左侧斜率范围的直线
  double min_angle = -90; // 最小角度
  double max_angle = -18; // 最大角度
  for (size_t i = 0; i < lines.size(); i++) {
    cv::Vec4i line = lines[i];
    double angle_rad = atan2(line[3] - line[1], line[2] - line[0]);
    double angle_deg = angle_rad * 180 / CV_PI;
    if (angle_deg >= min_angle && angle_deg <= max_angle) {
      filtered_lines.push_back(line);
    }
  }
  // 绘制左侧直线
  for (size_t i = 0; i < filtered_lines.size(); i++) {
    const cv::Vec4i &line = filtered_lines[i];
    cv::line(line_image, cv::Point(line[0], line[1]),
             cv::Point(line[2], line[3]), cv::Scalar(255), 2, cv::LINE_AA);
  }
  // 筛选右侧斜率范围的直线
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
  // 绘制右侧直线
  for (size_t i = 0; i < filtered_lines.size(); i++) {
    const cv::Vec4i &line = filtered_lines[i];
    cv::line(line_image, cv::Point(line[0], line[1]),
             cv::Point(line[2], line[3]), cv::Scalar(255), 2, cv::LINE_AA);
  }

  // 5. 最终处理与调用扫线
  cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
  cv::Mat dilated_ca2;
  cv::dilate(line_image, dilated_ca2, kernel2, cv::Point(-1, -1),
             1); // 对绘制的直线图像进行膨胀，使其更清晰

  // 调用核心处理函数，进行扫线、元素识别和误差计算
  int car_error = Image_Handle22(dilated_ca2, hsv_image, dilated_ca);

  // 调试图像显示
  if (saidao == 1) {
    cv::imshow("dilated_ca2", dilated_ca2); // 显示最终用于扫线的图像
    cv::imshow("膨胀后的ca", dilated_ca);   // 显示Canny+膨胀的图像
  }
  if (saidao1 == 1) {
    cv::imshow("膨胀后的ca", dilated_ca);
  }
  if (saidao == 3) {
    cv::imshow("膨胀后的ca", dilated_ca);
  }

  return car_error;
}

char element_flag = 26;     // 元素标志
char Change_track_flag = 1; // 换道标志
int flag_stop_zhuangpaizi = 0;

// 扫线相关全局变量
int bizhanground = 0;  // 避障计数器
int bizhangenable = 1; // 避障功能使能: 1-允许, 0-不允许
int yellowenable = 0;  // 黄线停车功能使能
int BZget;             // 避障判断结果
int stopbanma = 0;     // 斑马线停车状态标志

// error突变保护相关变量
int prev_error = 0;             // 上一帧的error值
bool error_freeze_flag = false; // 冻结标志（true：处于冻结状态）
int frozen_error_val = 25;      // 冻结时保持的error值

/**
 * @brief   处理error值的突变，防止方向剧烈变化
 * @param   current_error 当前计算的error值
 * @return  int 处理后的稳定error值
 * @note    当error从一个较大的正值突然变为负数时（通常发生在出弯时），
 *          会暂时“冻结”error在一个稳定值，直到它恢复正常范围，以避免车身过度摆动。
 */
int stabilize_error(int current_error) {
  // 1. 判断是否触发冻结条件：上一帧error较大，当前帧突然变为负数
  if (!error_freeze_flag && prev_error >= 6 && current_error <= 0) {
    error_freeze_flag = true;      // 触发冻结
    frozen_error_val = prev_error; // 冻结在上一帧的值
    std::cout << "触发error冻结！冻结值：" << frozen_error_val << std::endl;
    return frozen_error_val;
  }

  // 2. 若已冻结，判断是否解除冻结：当前error恢复到正值
  if (error_freeze_flag) {
    if (current_error >= 0) {
      error_freeze_flag = false; // 解除冻结
      std::cout << "解除error冻结！当前值：" << current_error << std::endl;
      prev_error = current_error; // 更新历史值
      return current_error;
    } else {
      // 未满足解除条件，继续返回冻结值
      return frozen_error_val;
    }
  }

  // 3. 正常状态：直接返回当前error，并更新历史值
  prev_error = current_error;
  return current_error;
}

/**
 * @brief 核心图像处理函数
 * @param data 用于扫线的二值化图像 (320x120)
 * @param YUANTU 原始HSV图像，用于颜色识别
 * @param BANMA Canny边缘图，用于斑马线识别
 * @return int 计算出的最终循迹误差
 * @note
 * 该函数协调了赛道线搜索、中线计算、元素（障碍物、斑马线、黄线）识别和最终误差计算。
 */
int Image_Handle22(cv::Mat data, cv::Mat YUANTU, cv::Mat BANMA) {
  static int BZ_con[9]; // 避障检测结果队列，用于滤波
  int errroer_car = 0;
  int16 i;
  // 初始化
  Line_Count = 0;
  Left_Add_Start = 0;
  Right_Add_Start = 0;
  Left_Add_Stop = 0;
  Right_Add_Stop = 0;
  for (i = ROW - 1; i >= 9; i -= 2) {
    Left_Add_Flag[i] = 1;
    Right_Add_Flag[i] = 1;
  }

  /***************************** 扫线与中线计算 **************************/
  int y = First_Line_Handle(data); // 获取虚拟首行中点
  // 从图像底部向上逐行搜索边界
  for (i = ROW - 1; i >= 9; i -= 2) {
    Line_Count = i;
    Earge_Search_Mid(i, data, Mid_Line[i + 2], 1, COL - 1, Left_Line,
                     Right_Line, Left_Add_Line, Right_Add_Line, 0);
  }
  LinearInterpolation(); // 中线线性插值平滑

  // 调试显示
  if (XUNJI_Imageflag == 1) {
    for (i = ROW - 1; i >= 9; i -= 2) {
      // 在图像上绘制左右边界点和中线点
      cv::circle(data, cv::Point(Right_Add_Line[i] - 5, i), 5, 255);
      cv::circle(data, cv::Point(Left_Add_Line[i] + 5, i), 8, 255);
      cv::circle(data, cv::Point(Interpolated_Liness[i], i), 1, 255);
    }
    cv::Mat kjkjkj;
    cv::resize(data, kjkjkj, cv::Size(), 0.5, 0.5);
    cv::imshow("循迹", kjkjkj); // 显示最终的扫线结果图
  }

  /***************************** 元素处理 **************************/
  if (yellowenable == 1) {
    yellow_chuli(YUANTU); // 黄线停车处理
  }

  if (bizhangenable == 1) // 避障处理
  {
    int BZ_GET = 0;
    // 将当前帧的检测结果存入队列
    BZ_con[8] = BZ_chuli(YUANTU);
    for (int k = 0; k < 8; ++k)
      BZ_con[k] = BZ_con[k + 1];
    BZ_con[7] = BZ_con[8];

    // 统计队列中有多少次检测到障碍物
    for (int i = 0; i <= 7; i++) {
      if (BZ_con[i] == 1)
        BZ_GET++;
    }

    if (BZ_GET >= 2) // 如果连续多帧检测到，则认为是有效障碍物
    {
      BZget = BZ_PANDUAN_2(); // 判断障碍物是否在赛道内
      BS_BZ_FLAG = BZget;
      if (BZget == 1) // 在赛道内
      {
        cv::circle(data, cv::Point(find_XYdata[0], find_XYdata[1]), 10,
                   255); // 标记障碍物位置
        std::cout << "有障碍物在赛道内" << std::endl;
        BZ_LuoJISET(); // 执行避障逻辑
        bizhanground++;
      } else
        std::cout << "有障碍物在赛道外" << std::endl;
    }
  }

  if (banmaenable == 1) // 斑马线处理
  {
    BMGet = BanMa_Find111(BANMA);
    std::cout << "BMGet " << BMGet << std::endl;
  }

  if (BMGet == 1 && stopbanma == 0) // 检测到斑马线且尚未触发停车
  {
    stopbanma = 1; // 触发停车标志
  }

  /***************************** 误差计算 **************************/
  LinearInterpolation();     // 再次平滑可能被避障逻辑修改过的中线
  errroer_car = error_get(); // 计算加权误差
  return (errroer_car);
}

// 循迹误差加权表，不同行的中线偏移赋予不同权重
// 权重越大，该行的偏移对最终误差的影响越大
uint8 Weight_th[110] = {
    2, 2, 2, 2, 2, 3, 3, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 7, 7, 7, 7, 7, 7, 8, 8, 8, 9,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 2, 2, 2, 2, 2,
    2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
};

/**
 * @brief 计算循迹误差
 * @return int 加权平均后的循迹误差值
 * @note 将各行中线与图像中心的偏移量，根据Weight_th表进行加权求和，再求平均。
 *       最后对结果进行限幅并缩放。
 */
int error_get(void) {
  long error_in = 0;
  int error_out = 0;
  long Weight_Count = 0;
  int j = 0;
  for (int i = ROW - 1; i >= 9; i -= 2) {
    // 误差 = (中线位置 - 图像中心) * 权重
    error_in += (Mid_Line[i] - 320) * Weight_th[j];
    Weight_Count += Weight_th[j];
    j++;
  }
  error_out = error_in / Weight_Count; // 求加权平均
  // 限幅
  if (error_out < -160) {
    error_out = -160;
  }
  if (error_out > 160) {
    error_out = 160;
  }
  return error_out / 4; // 缩放后返回
}

// 调试图像显示标志
int BZ_Imageflag = 0, BM_Imageflag = 0, yellow_Imageflag = 0,
    blue_Imageflag = 0, XUNJI_Imageflag = 0, CA_Imageflag = 0;

///////////////////避障部分//////////////////////
int find_XYdata[3];        // 存储障碍物坐标 [x, y, 0]
int find_XYdata_second[3]; // 备用障碍物坐标

/**
 * @brief 避障补线函数
 * @param data_X 障碍物中心X坐标
 * @param data_Y 障碍物中心Y坐标 (未使用)
 * @param bizhang_fangxiang 避障方向: 0-向左, 1-向右
 * @note 根据避障方向，强制修改一侧的边界线，从而引导车辆绕开障碍物。
 */
void bizhangBuxian(int data_X, int data_Y, int bizhang_fangxiang) {
  if (bizhang_fangxiang == 0) // 左避障
  {
    // 将右边界线设置在障碍物左侧一个安全距离
    for (int i = ROW - 1; i >= 9; i -= 2) {
      Right_Add_Line[i] = data_X - Bizhang_line_move - 6;
    }
    std::cout << "向左" << std::endl;
  } else if (bizhang_fangxiang == 1) // 右避障
  {
    // 将左边界线设置在障碍物右侧一个安全距离
    for (int i = ROW - 1; i >= 9; i -= 2) {
      Left_Add_Line[i] = data_X + Bizhang_line_move + 6;
    }
    std::cout << "向右" << std::endl;
  }
}
cv::Mat red_mask; // 红色掩码图
/**
 * @brief 避障逻辑状态机
 * @note 通过一个简单的状态机判断障碍物是刚出现、即将经过还是已经经过，
 *       并交替选择左右避障方向。
 */
void BZ_LuoJISET(void) {
  static int zhangai_flag1 = 0;         // 状态机状态
  static int zhangai_flag22 = 1;        // 左右避障选择
  static int zhangai_Left_or_Right = 0; // 最终避障方向
  static int nums = 0;                  // 帧计数器，用于状态确认

  // 状态0: 等待障碍物在图像下方出现
  if (find_XYdata[1] > 40 && zhangai_flag1 == 0) {
    nums++;
    if (nums >= 1) // 连续1帧确认
    {
      nums = 0;
      zhangai_flag1 = 1; // 进入状态1
    }
  }
  // 状态1: 等待障碍物移动到图像上方 (即车辆即将经过)
  if (zhangai_flag1 == 1) {
    if (find_XYdata[1] <= 30) {
      nums++;
      if (nums >= 1) // 连续1帧确认
      {
        nums = 0;
        zhangai_flag1 = 2; // 进入状态2
      }
    }
  }
  // 状态2: 障碍物已经经过，切换下一次的避障方向，并重置状态机
  if (zhangai_flag1 == 2) {
    zhangai_flag22++;
    if (zhangai_flag22 >= 2)
      zhangai_flag22 = 0; // 0和1之间切换
    zhangai_flag1 = 0;    // 回到状态0
  }
  // 根据zhangai_flag22确定本次避障方向
  if (zhangai_flag22 == 0) {
    std::cout << "左避障" << std::endl;
    zhangai_Left_or_Right = 0;
  }
  if (zhangai_flag22 == 1) {
    std::cout << "右避障" << std::endl;
    zhangai_Left_or_Right = 1;
  }

  // 执行补线和中线修复
  bizhangBuxian(find_XYdata[0], find_XYdata[1], zhangai_Left_or_Right);
  Mid_Line_Repair();
}

int red1max = 0, red2max = 0, red3max = 0, red1min = 0, red2min = 0,
    red3min = 0; // HSV阈值变量，用于UI调节
/**
 * @brief 障碍物(蓝色/红色物体)检测函数
 * @param BZdata 输入的HSV格式图像
 * @return int 1-检测到障碍物, 0-未检测到
 * @note 通过颜色阈值分割、形态学操作和轮廓查找来定位障碍物。
 */
int BZ_chuli(cv::Mat BZdata) {
  cv::Mat hsvvvv;
  cv::cvtColor(cropped_imageddddd, hsvvvv, cv::COLOR_BGR2HSV); // BGR -> HSV

  // 定义蓝色/红色范围
  cv::Scalar blue_lower, blue_upper;
  if (red_set == 0) // 使用预设的蓝色阈值
  {
    blue_lower = cv::Scalar(136, 150, 49);
    blue_upper = cv::Scalar(179, 255, 255);
  } else // 使用UI滑块调节的阈值
  {
    blue_lower = cv::Scalar(Hmin, Smin, Vmin);
    blue_upper = cv::Scalar(Hmax, Smax, Vmax);
  }

  cv::Mat blue_mask;
  cv::inRange(hsvvvv, blue_lower, blue_upper,
              blue_mask); // 颜色阈值分割，生成二值掩码图

  // 形态学操作，去除噪声，填充空洞
  cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
  cv::dilate(blue_mask, blue_mask, kernel2, cv::Point(-1, -1), 1);
  cv::dilate(blue_mask, blue_mask, kernel2, cv::Point(-1, -1), 1);

  // 调试显示
  if (red_set == 1) {
    cv::imshow("BZdata", blue_mask);
  }
  cv::waitKey(10);

  // 寻找轮廓
  std::vector<std::vector<cv::Point>> red_contours;
  cv::findContours(blue_mask, red_contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  // 找到最大的轮廓
  double max_area = 0;
  int max_area_index = -1;
  for (int i = 0; i < red_contours.size(); i++) {
    double area = cv::contourArea(red_contours[i]);
    if (area > max_area) {
      max_area = area;
      max_area_index = i;
    }
  }

  // 如果找到了足够大的轮廓
  if (max_area_index >= 0) {
    if (max_area <= 2)
      return 0; // 面积太小，认为是噪声，返回0

    // 计算最大轮廓的中心点坐标
    cv::Rect bounding_rect = cv::boundingRect(red_contours[max_area_index]);
    int center_x = (bounding_rect.x + bounding_rect.width / 2) / 2;
    int center_y = (bounding_rect.y + bounding_rect.height / 2) / 2;

    // 保存坐标并返回1
    find_XYdata[0] = center_x;
    find_XYdata[1] = center_y;
    return 1;
  }

  return 0; // 未找到障碍物
}
/**
 * @brief 黄色物体(停车标志)检测函数
 * @param BZdata 输入的HSV格式图像
 * @return int 总是返回0，实际停车标志在CAR_STOP中设置
 * @note 流程与BZ_chuli类似，但针对黄色。
 */
int yellow_chuli(cv::Mat BZdata) {
  cv::Mat hsvvvv;
  cv::cvtColor(cropped_imageddddd, hsvvvv, cv::COLOR_BGR2HSV);

  // 定义黄色范围
  cv::Scalar yellow_lower(142, 45, 55);
  cv::Scalar yellow_upper(179, 255, 255);

  cv::Mat yellow_mask;
  cv::inRange(BZdata, yellow_lower, yellow_upper, yellow_mask);

  // 形态学操作
  cv::Mat kerneyellow =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
  cv::erode(yellow_mask, yellow_mask, kerneyellow, cv::Point(-1, -1), 1);
  cv::Mat kerneyellow22 =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
  cv::dilate(yellow_mask, yellow_mask, kerneyellow22, cv::Point(-1, -1), 1);

  // 调用停车判断函数
  CAR_STOP_FLAG = CAR_STOP(yellow_mask);

  // 调试显示
  if (yellow_set == 1) {
    cv::imshow("BZdata", yellow_mask);
    cv::waitKey(10);
  }
  return 0;
}

/**
 * @brief 判断障碍物是否在赛道内
 * @return int 1-在赛道内, 0-在赛道外
 * @note
 * 根据障碍物的Y坐标找到它所在的大致行号，然后比较其X坐标是否在当前行的左右边界之间。
 */
int BZ_PANDUAN_2(void) {
  int Y = 0;
  if (find_XYdata[1] <= 10)
    return 0; // 障碍物太靠上，忽略
  // 找到障碍物Y坐标对应的图像行
  for (int i = ROW - 1; i >= 9; i -= 2) {
    if (find_XYdata[1] >= i) {
      Y = i;
      break;
    }
    if (i <= 11)
      return 0; // 障碍物太靠下，无法判断，忽略
  }
  // 判断X坐标是否在左右边界之间（包含一定裕量）
  if (find_XYdata[0] <= Left_Add_Line[Y] - 25 ||
      find_XYdata[0] >= Right_Add_Line[Y] + 25) {
    return 0; // 在赛道外
  } else
    return 1; // 在赛道内
}

////////////////////////////
//---------斑马线---------//
////////////////////////////
/**
 * @brief 斑马线检测函数
 * @param BanMa_Find_data 输入的二值化图像 (通常是Canny边缘图)
 * @return int 1-检测到斑马线, 0-未检测到
 * @note 通过查找轮廓，并根据轮廓的尺寸和位置筛选出可能是斑马线块的矩形。
 *       如果一帧内找到足够数量的斑马线块，并且它们的位置分布合理，则认为检测到斑马线。
 */
int BanMa_Find111(cv::Mat BanMa_Find_data) {
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(BanMa_Find_data, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  cv::Mat contour_img = BanMa_Find_data.clone();
  static int count_BMXduilie[8]; // 斑马线检测结果队列
  int Y_point[20];               // 存储斑马线块的Y坐标
  int count_BMX = 0;             // 当前帧检测到的斑马线块数量

  // 定义斑马线块的尺寸阈值
  int min_wh = 5;
  int max_wh = 55;

  // 遍历所有轮廓
  for (const auto &contour : contours) {
    cv::Rect rect = cv::boundingRect(contour); // 获取轮廓的外接矩形
    // 1. 尺寸筛选
    if (min_wh <= rect.height && rect.height < max_wh && min_wh <= rect.width &&
        rect.width < max_wh) {
      // 2. 位置筛选 (Y坐标在有效范围内)
      if (rect.y >= 10 && rect.y <= 85) {
        if (rect.y % 2 == 0)
          rect.y = rect.y - 1; // 统一到奇数行
        // 3. 位置筛选 (X坐标在赛道内)
        if (rect.y >= 85) // 靠近底部的行，使用固定X范围
        {
          if (rect.x >= (20) && rect.x <= (300)) {
            cv::rectangle(contour_img, rect, cv::Scalar(255), 2);
            count_BMX++;
            Y_point[count_BMX] = rect.y;
          }
        } else // 其他行，使用动态的赛道边界
        {
          if (rect.x >= (Left_Add_Line[rect.y] - 20) &&
              rect.x <= (Right_Add_Line[rect.y] + 20)) {
            cv::rectangle(contour_img, rect, cv::Scalar(255), 2);
            count_BMX++;
            Y_point[count_BMX] = rect.y;
          }
        }
      }
      if (count_BMX >= 6)
        count_BMX = 6; // 最多记录6个
    }
  }

  // 如果当前帧找到足够多的斑马线块
  if (count_BMX >= 4) {
    // 计算这些块Y坐标的离散程度
    int pingJun = (Y_point[1] + Y_point[2] + Y_point[3] + Y_point[4]) / 4;
    pingJun = (Q_jdz(pingJun - Y_point[1]) + Q_jdz(pingJun - Y_point[2]) +
               Q_jdz(pingJun - Y_point[3]) + Q_jdz(pingJun - Y_point[4])) /
              4;

    banmaxian_Y = (Y_point[1] + Y_point[2] + Y_point[3] + Y_point[4]) /
                  4; // 计算平均Y坐标
    count_BMX = 0;
    count_BMXduilie[7] = 1; // 将检测结果存入队列

    // 如果离散程度太大，说明这些块可能不是同一排斑马线，认为是无效检测
    if (pingJun >= 15) {
      return 0;
    }
  } else {
    count_BMX = 0;
    count_BMXduilie[7] = 0;
  }

  // 队列滚动
  for (int k = 0; k < 7; ++k)
    count_BMXduilie[k] = count_BMXduilie[k + 1];

  // 统计队列中检测到斑马线的次数
  for (int num = 0; num <= 7; num++) {
    if (count_BMXduilie[num] == 1) {
      count_BMX++;
    }
  }

  if (banma111 == 1) {
    cv::imshow("contour_img", contour_img);
  }

  // 如果连续多帧检测到，则最终确认检测到斑马线
  if (count_BMX >= 3) {
    return 1;
  } else
    return 0;
}

/////////////////////////
//---------------------//
/////////////////////////

/**
 * @brief 斑马线检测函数 (备用方案)
 * @param BanMa_Find_data 输入的原始BGR图像
 * @return int 1-检测到, 0-未检测到
 * @note
 * 与BanMa_Find111不同，此函数内部自己进行灰度化、滤波、边缘检测等一系列预处理。
 */
int BanMa_Find(cv::Mat BanMa_Find_data) {
  // 定义感兴趣区域
  Rect roi(100, 0, 120, 240);
  BanMa_Find_data = BanMa_Find_data(roi);
  // 预处理
  cv::resize(BanMa_Find_data, BanMa_Find_data, cv::Size(), 1, 0.5);
  cv::Mat gray_image;
  cv::cvtColor(BanMa_Find_data, gray_image, cv::COLOR_BGR2GRAY);
  cv::Mat blur;
  cv::bilateralFilter(gray_image, blur, 7, 60, 60);
  cv::Mat gaussian_blur;
  cv::GaussianBlur(blur, gaussian_blur, cv::Size(5, 5), 30);
  cv::Mat ca;
  cv::Canny(gaussian_blur, ca, 30, 50);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
  cv::Mat dilated_ca;
  cv::dilate(ca, dilated_ca, kernel, cv::Point(-1, -1), 1);
  // 查找轮廓
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(dilated_ca, contours, cv::RetrievalModes::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  cv::Mat contour_img = dilated_ca.clone();
  std::vector<int> Y_points;
  int count_BMX = 0;
  const int min_wh = 5;
  const int max_wh = 55;
  // 遍历轮廓并筛选
  for (const auto &contour : contours) {
    cv::Rect rect = cv::boundingRect(contour);
    if (rect.height >= min_wh && rect.height < max_wh && rect.width >= min_wh &&
        rect.width < max_wh) {
      if (rect.y >= 40 && rect.y <= 72) {
        if (rect.y % 2 == 0)
          rect.y -= 1;
        if (rect.y >= 85 || (rect.x >= 20 && rect.x <= 300)) {
          cv::rectangle(contour_img, rect, cv::Scalar(255, 0, 0), 2);
          Y_points.push_back(rect.y);
          ++count_BMX;
        }
      }
    }
  }
  // 判断
  if (count_BMX >= 4) {
    int sum_Y = std::accumulate(Y_points.begin(), Y_points.end(), 0);
    int average_Y = sum_Y / count_BMX;
    int pingJun = std::accumulate(Y_points.begin(), Y_points.end(), 0,
                                  [average_Y](int acc, int y) {
                                    return acc + std::abs(average_Y - y);
                                  }) /
                  count_BMX;
    if (pingJun < 5) {
      std::cout << "斑马线" << std::endl;
      return 1;
    }
  }

  if (banma222 == 1) {
    cv::imshow("contour_img", contour_img);
  }
  return 0;
}

////////////////////////////
//---------黄线停车--------//
////////////////////////////

/**
 * @brief 黄线停车判断函数
 * @param yellow_Find_data 输入的黄色掩码图 (320x96)
 * @return int 1-需要停车, 0-不需要停车
 * @note
 * 在图像底部的一个特定区域内，统计黄色像素点的数量。如果数量超过阈值，则认为检测到黄线，需要停车。
 */
int CAR_STOP(cv::Mat yellow_Find_data) {
  int yellowPoint_num = 0;
  // 在图像底部 30x289 的区域内扫描
  for (int i = 95; i >= 95 - 30; i -= 1) {
    for (int j = 319 - 30; j >= 30; j -= 2) {
      if (yellow_Find_data.at<uchar>(i, j) > 100) // 如果是黄色像素
      {
        yellowPoint_num++;
      }
    }
  }
  // 如果黄色像素点数大于阈值
  if (yellowPoint_num >= 80) {
    return 1; // 返回停车信号
    std::cout << "已检测到黄色线" << std::endl;
  }

  return 0;
}
/////////////////////////
//---------------------//
/////////////////////////

void onTrackbar(int, void *) {
  // This function is called whenever a trackbar is moved.
  // You can access the current values of the trackbars here.
}
int Hmin = 0, Hmax = 0, Smin = 0, Smax = 0, Vmin = 0, Vmax = 0;
void UI_init(void) {
  namedWindow("TrackBars");
  resizeWindow("TrackBars", 640, 640);

  // Create trackbars for HSV values
  // createTrackbar("遥控", "TrackBars", &car_l_mid_stop, 4, onTrackbar);
  createTrackbar("Hue Min", "TrackBars", &Hmin, 179, onTrackbar);
  createTrackbar("Hue Max", "TrackBars", &Hmax, 179, onTrackbar);
  createTrackbar("Sat Min", "TrackBars", &Smin, 255, onTrackbar);
  createTrackbar("Sat Max", "TrackBars", &Smax, 255, onTrackbar);
  createTrackbar("Val Min", "TrackBars", &Vmin, 255, onTrackbar);
  createTrackbar("Val Max", "TrackBars", &Vmax, 255, onTrackbar);
  createTrackbar("red_set ", "TrackBars", &red_set, 1, onTrackbar);
  createTrackbar("yellow_set ", "TrackBars", &yellow_set, 1, onTrackbar);
  createTrackbar("saidao ", "TrackBars", &saidao, 1, onTrackbar);
  createTrackbar("saidao1 ", "TrackBars", &saidao1, 1, onTrackbar);
  createTrackbar("banma111 ", "TrackBars", &banma111, 1, onTrackbar);
  createTrackbar("banma222 ", "TrackBars", &banma222, 1, onTrackbar);
}
