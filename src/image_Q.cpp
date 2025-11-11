#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include "image_Q.hpp"
#include <numeric>
extern "C" 
{
	#include "serial.h"
}
int car_speed = setspeed1;    
int banmaxian_Y = 0;//斑马线坐标
int16  EdgeThres = 18;                 //跳变沿阈值  18  17  ////
float  BlackThres = 80;          //黑白阈
cv::Mat  Imagel;                 //黑白阈值
int16  Left_Line[ROW + 2], Right_Line[ROW + 2];//左右边界（储存对应图像上的横坐标）、、、、、
int16  Mid_Line[ROW + 2];                     //赛道中线
int16  Left_Add_Line[ROW + 2], Right_Add_Line[ROW + 2];     // 左右边界补线数据
int16  Left_Add_Flag[ROW + 2], Right_Add_Flag[ROW + 2];        // 左右边界补线标志
int16  Road_Width_Real[ROW + 2];  // 实际赛道宽度
int16  Road_Width_Add[ROW + 2];   // 补线赛道宽度
int16  island_flag = 0;  //标志0不是环岛   标志1是左环岛   标志2是右环岛
int16  Road_Width_Min;      // 最小赛道宽
int16 Left_Add_Start, Right_Add_Start;  // 左右补线起始行坐标
int16 Left_Add_Stop, Right_Add_Stop;    // 左右补线结束行坐标
int16 Line_Count;   // 记录成功识别到的赛道行数
int16 Out_Side = 0; // 丢线控制
float Left_Ka = 0, Right_Ka = 0;
float Left_Kb = 1, Right_Kb = COL - 1;  // 最小二乘法参数

int BS_BZ_FLAG = 0;//避障变速
int CAR_STOP_FLAG = 0;//停车
int TIME_BIZHANG = 1;//到时间后打开臂章并减速
//-------------------------------------------------------------------------------------
int BMGet = 0;//斑马线标志
int BZ_FLAG_FLAG = 0;
int banmaenable=1;//2是不允许，1是允许  
//-------------------------------------------------------------------------------------
int CAR_FaChe(cv::Mat FaChe_data);//发车
int BanMa_Find111(cv::Mat BanMa_Find_data);
//-------------------------------------------------------------------------------------
int red_set = 0;
int yellow_set = 0;
int saidao = 0;
int saidao1 = 0;
int banma111 = 0;
int banma222 = 0;


/*-------------------------------------------------------------------------------------------------------------------
函数：限幅保护
说明：补线时坐标不能小于0，大于边界值
-------------------------------------------------------------------------------------------------------------------*/
int16 Limit_Protect(int16 num, int32 min, int32 max)
{
    if (num >= max)
        return max;
    else if (num <= min)
        return min;
    else
        return num;
}
/*-------------------------------------------------------------------------------------------------------------------
函数：计算补线坐标(Poit = Ka * i + Kb)
说明：先用两点法确定Ka,Kb然后计算出第i行的补线横坐标
-------------------------------------------------------------------------------------------------------------------*/
int16 Fit_Point(uint8 i, float Ka, float Kb)
{
    float res;
    int16 Result;
    res = i * Ka + Kb;
    Result = Limit_Protect((int16)res, 1, COL - 1);
    return Result;
}
/*-------------------------------------------------------------------------------------------------------------------
函数：求绝对值
说明：
-------------------------------------------------------------------------------------------------------------------*/
char Error_Transform(uint8 Data, uint8 Set_num)
{
    char Error;

    Error = Set_num - Data;
    if (Error < 0)
    {
        Error = -Error;
    }

    return Error;
}

int Q_jdz(int A)
{
    if (A < 0) A = -A;

    return A;
}

/*-------------------------------------------------------------------------------------------------------------------
函数：曲线拟合1
说明：先用两点法确定Ka,Kb然后计算出第i行的补线横坐标
            拟合直线 y = Ka * x + Kb   Mode == 1代表左边界，Mode == 2代表右边界
-------------------------------------------------------------------------------------------------------------------*/
void Curve1_Fitting(float* Ka, float* Kb, int16* Start, int16* Line_Add, int16 Mode)
{
    int i;                                      //用于内部循环
    int _start;                             //临时变量储存起始�?
    *Start += 2;                            //i行已经需要补线肯定找前一行数据此行扫描到边界
    _start = *Start;
    if (Mode == 2)                      //右补�?
    {
        for (i = _start; i <= _start + 6; i += 2)                            //寻找右边界近处突出点
        {
            if (Right_Line[i] < Right_Line[*Start])
                *Start = i;  //更新突出�?
        }
        if (*Start >= 59)   *Start = 57;

        *Ka = 1.0 * (Line_Add[*Start + 2] - Line_Add[*Start]) / 2;  //计算Ka
        if (*Ka < 0)                                                                        //防止出现负值，我觉得这个用不到�?
            *Ka = 0;
    }
    else if (Mode == 1)
    {
        for (i = _start; i <= _start + 6; i += 2)                            //寻找左边界近处突出点
            if (Left_Line[i] > Left_Line[*Start])
                *Start = i;                                                                 //更新突出�?
        if (*Start >= 59)
            *Start = 57;
        *Ka = 1.0 * (Line_Add[*Start + 2] - Line_Add[*Start]) / 2;  //计算Ka
        if (*Ka > 0)                                                                        //防止出现负值，我觉得这个用不到�?
            *Ka = 0;
    }
    *Kb = 1.0 * Line_Add[*Start] - (*Ka * (*Start));                      //代入公式计算Kb
}
/*-------------------------------------------------------------------------------------------------------------------
函数：曲线拟合2
说明：先用两点法确定Ka,Kb然后计算出第i行的补线横坐标
            拟合直线 y = Ka * x + Kb   Mode == 1代表左边界，Mode == 2代表右边界
-------------------------------------------------------------------------------------------------------------------*/
void Curve2_Fitting(float* Ka, float* Kb, uint8 Start, uint8 End, int16* Line, int16 Mode, int16 num)
{
    if (Mode == 1)
    {
        *Ka = 1.0 * ((Line[Start] + num) - Line[End]) / (Start - End);
        *Kb = 1.0 * Line[End] - (*Ka * End);
    }
    else
    {
        *Ka = 1.0 * ((Line[Start] - num) - Line[End]) / (Start - End);
        *Kb = 1.0 * Line[End] - (*Ka * End);
    }
}
/*-------------------------------------------------------------------------------------------------------------------
函数：曲线拟合3
说明：先用两点法确定Ka,Kb然后计算出第i行的补线横坐标
            拟合直线 y = Ka * x + Kb   Mode == 1代表左边界，Mode == 2代表右边界
-------------------------------------------------------------------------------------------------------------------*/
void Curve3_Fitting(float* Ka, float* Kb, uint8 Start, uint8 End, int16* Line, int16 Mode)//环岛检测专用，最正规的求�?
{
    if (Mode == 1)
    {
        *Ka = 1.0 * ((Line[Start]) - Line[End]) / (Start - End);
        *Kb = 1.0 * Line[End] - (*Ka * End);
    }
    else
    {
        *Ka = 1.0 * ((Line[Start]) - Line[End]) / (Start - End);
        *Kb = 1.0 * Line[End] - (*Ka * End);
    }
}

/*-------------------------------------------------------------------------------------------------------------------
函数：边界处理函数，从中间向两边搜索边界
说明：本函数使用后将保存边界数据，丢线只更新Add_Line的值不更新Line的值�?
            i 外部变量用于行循�?          data获取压缩后的图像Imagel
            Mid上一行中线的值（调用i+1�?  Left_Min左边最小�?      Right_Max右边最大�?
            Left_Line实际左边�?           Right_Line实际右边�?    Left_Add_Line补线左边�?      Right_Add_Line补线右边�?
-------------------------------------------------------------------------------------------------------------------*/
void Earge_Search_Mid(int16 i, cv::Mat data, int16 Mid, int16 Left_Min, int16 Right_Max, int16* Left_Line, int16* Right_Line, int16* Left_Add_Line, int16* Right_Add_Line, int mods)
{
    int16 j;                                            //用于内部列循�?
    int16 N = 6;                        /*前N行丢线特殊处理，近处丢线如果不特殊处理，补线过于偏的话，因为所占权重大，会导致的影响比较大*/

    Left_Add_Flag[i] = 1;              //初始化补线标志位�?0为不需要补线，1为需要补�?
    Right_Add_Flag[i] = 1;

    Right_Line[i] = Right_Max;          //给定边界初始值，一般为1和COL-1
    Left_Line[i] = Left_Min;
int16 avg_width = 0;
int valid_width_count = 0;
for (int k = ROW-1; k >= 9; k -= 2) {
    if (Road_Width_Real[k] > 50 && Road_Width_Real[k] < 300) {  // 过滤异常值
        avg_width += Road_Width_Real[k];
        valid_width_count++;
    }
}
if (valid_width_count > 0) avg_width /= valid_width_count;
else avg_width = 150;  // 默认宽度（根据实际场景调整）

    /*左边线查�?*/
    for (j = Mid; j >= 10; j-=4)                         //以前一行中点为起点向左查找边界
    {
        if ((data.at<uchar>(i, j) < 100)&&(data.at<uchar>(i, j-4) < 100)&&(data.at<uchar>(i, j-8) > 100))                     /*黑白�?*//*为啥不用全用阈值写if(data[i][j] < BlackThres  && data[i][j-1] < BlackThres)�?*/
        {/*上面的BlackThres后面的数字可以根据需要调一�?*/
            if(j>= 320/2 + 50)
            {
                j = j-30;
                 //继续�?
            }
            else 
            {
                    Left_Add_Flag[i] = 0;           //左边界不需要补线，清除标志�?
                    Left_Line[i] = j;       //记录当前j值为本行实际左边�?
                    Left_Add_Line[i] = j;           //记录实际左边界为补线左边�?
                    break;  
            }
                                               //找到退�?
        }
    }
    /*右边线查�?*/
    for (j = Mid; j <= COL - 10; j+=4)    // 以前一行中点为起点向右查找右边�?
    {
        if ((data.at<uchar>(i, j) < 100)&&(data.at<uchar>(i, j+4) < 100)&&(data.at<uchar>(i, j+8) > 100))
        {/*上面的BlackThres后面的数字可以根据需要调一�?*/
          if(j<= 320/2 - 50)
            {
                j = j+30;
                 //继续�?
            }
            else 
            {
                 Right_Add_Flag[i] = 0;      //右边界不需要补线，清除标志�?
                 Right_Line[i] = j;      //记录当前j值为本行右边�?
                 Right_Add_Line[i] = j;      //记录实际右边界为补线左边�?
                 break;     
            }                                 //找到退�?
        }
    }


    if (Left_Add_Flag[i])                    //�?1表示：没找到左边界需要补�?
    {   
        //std::cout << "===== 补左线了=====" << std::endl;
        if (i >= ROW - N)
            //Left_Add_Line[i] = Left_Line[ROW - 1];        //使用底行数据的左边界数据作为本行左边界，？底行特别容易丢线？
            Left_Add_Line[i] = Right_Line[ROW - 1] -  200; 
        else
           // Left_Add_Line[i] = Left_Add_Line[i + 2];      //使用前此行之前的2行的左边界数据作为本行左边界
            Left_Add_Line[i] = Right_Add_Line[ROW - 1] -  200; 

    }
    /*右边线补线处理，注意只更新Right_Add_Line数组，不更新Right_Line数组*/
    if (Right_Add_Flag[i])                   //�?1表示：没找到右边界需要补�?
    {   
        //std::cout << "===== 补右线了=====" << std::endl;
        if (i >= ROW - N)  
            {                       //�?6行特殊处�?
            //Right_Add_Line[i] = Right_Line[ROW - 1];      //使用底行数据的右边界数据作为本行右边界，？底行特别容易丢线？
            Right_Add_Line[i] = Left_Line[ROW - 1] + 200;    
            }
            else
            //Right_Add_Line[i] = Right_Add_Line[i + 2];    //使用前此行之前的2行的右边界数据作为本行右边界
        {Right_Add_Line[i] = Left_Add_Line[i + 2] + 200;}
            
    }

    Road_Width_Real[i] = Right_Line[i] - Left_Line[i];          //计算实际赛道宽度
    Road_Width_Add[i] = Right_Add_Line[i] - Left_Add_Line[i];  //计算补线赛道宽度，在TFT屏幕上显示的是这个边界线 
//    printQ("赛道宽度",Road_Width_Add[i]);
//    printQ("第几�?",i);
    Mid_Line[i] = (Right_Add_Line[i] + Left_Add_Line[i]) / 2; //�?
}
//首行处理=====================================================================================================================================
int16 First_Line_Handle(cv::Mat data)
{
    static long mind_jundata[5];
    Mid_Line[ROW + 1] = Mid_Line[ROW - 1];
    if (Mid_Line[ROW + 1] >= 320 - 40 || Mid_Line[ROW + 1] <= 40)
    {
         Mid_Line[ROW + 1] = COL / 2;
    }
//    // printQ("起始找中线位�?", Mid_Line[ROW + 1]);
    return 0;                //返还1 表示成功
}
/*-------------------------------------------------------------------------------------------------------------------
函数：中线修补
说明：放到最后更新补线完后后中线
            count为Line_Count有效行数         data获取的图像Imagel
-------------------------------------------------------------------------------------------------------------------*/
void Mid_Line_Repair()//中线修复
{
    for (int i = ROW-1; i >= 9; i -= 2)  //从第一行到截至�?
    {
        Mid_Line[i] = (Right_Add_Line[i] + Left_Add_Line[i]) / 2; //中线
    }
}
int Interpolated_Liness[ROW + 2]; //  插值后的数组
// 线性化
void LinearInterpolation(void)
{
    float data = 0;
     //线性插值
    for (int i = ROW- 1; i >= 9; i-=2)
    {
        Interpolated_Liness[i] = (Mid_Line[i] + Mid_Line[i - 2]) / 2;
        Mid_Line[i - 2] = Interpolated_Liness[i];
    }

}
cv::VideoWriter writer("output288.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 60, cv::Size(320, 96));  // 创建视频写入对象，指定输出文件、编码器、帧率和分辨�?
cv::Mat cropped_imageddddd; 
cv::Mat banmachuli;

int count_valid_lines(int line_array[], int total_rows) {
    int valid_count = 0;
    for (int i = 0; i < total_rows; i++) {
        // 假设“无效坐标”是0或超出图像宽度（比如图像宽度320，x>320则无效）
        if (line_array[i] > 0 && line_array[i] < 320) { // 根据你的图像宽度调整（320是示例）
            valid_count++;
        }
    }
    return valid_count;
}
int right_valid_count;
int left_valid_count;
int TUxiang_Init(cv::Mat data)//图像预处理
{
    // 将图像的下半部分进行裁剪   240  640
     banmachuli=data;
    cv::resize(banmachuli, banmachuli, cv::Size(), 0.5, 0.5);
    cv::Rect roi_rect(0, (data.rows / 2 - 90 + 70), data.cols, (data.rows / 2.5));
  //  cv::Rect roi_rect(0, (data.rows / 2 - 90 + 70 - 5), data.cols, (data.rows / 2.5));
    cv::Mat cropped_image = data(roi_rect); 
    cropped_imageddddd = cropped_image;
    // 压缩图像尺寸
    if (!cropped_image.empty())
{
    cv::resize(cropped_image, cropped_image, cv::Size(), 0.5, 0.5);
}
else
{
    std::cout << "Cropped image is empty!" << std::endl;
}
   writer.write(cropped_image);  // 将帧图像写入视频文件
    if(blue_Imageflag == 2)  cv::imshow("frame", cropped_image);

    // 将图像从 BGR（蓝绿红）颜色空间转换为 HSV（色相饱和度明度）颜色空间
    //std::cout << "图像尺寸: " << cropped_image.size() << std::endl;
    cv::Mat hsv_image;
    cv::cvtColor(cropped_image, hsv_image, cv::COLOR_BGR2HSV);
    // 将图像从 BGR（蓝绿红）颜色空间转换为灰度颜色空间
    cv::Mat gray_image;
    cv::cvtColor(cropped_image, gray_image, cv::COLOR_BGR2GRAY);
    cv::Mat blur;
    // 对灰度图进行双边滤波
    cv::bilateralFilter(gray_image, blur, 7, 60,60);
    cv::Mat gaussian_blur;
    // 对模糊后的图像进行高斯滤波
    cv::GaussianBlur(blur, gaussian_blur, cv::Size(5, 5), 30);
   // cv::imshow("blur", gaussian_blur);//裁剪后的原图
    // 使用Canny算子进行边缘检测
    cv::Mat ca;
    cv::Canny(gaussian_blur, ca, 30, 50);
    // 定义一个膨胀核
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    // 对边缘检测后的图像进行膨胀处理
    cv::Mat dilated_ca;
    cv::dilate(ca, dilated_ca, kernel, cv::Point(-1, -1), 1);
    // 使用Hough线段检测算法检测线条
    std::vector<cv::Vec4i> lines;
     cv::HoughLinesP(dilated_ca, lines, 1, CV_PI / 180, 70, 25, 5);

    cv::Mat line_image = cv::Mat::zeros(dilated_ca.size(), CV_8UC1);
    // 过滤指定范围外的角度=============《《《开始》
    std::vector<cv::Vec4i> filtered_lines;
    //先画左线
    double min_angle = -90;  // 最小角度（以度为单位）
    double max_angle = -18;  // 最大角度（以度为单位）

    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i line = lines[i];
        double angle_rad = atan2(line[3] - line[1], line[2] - line[0]);  // 计算角度（弧度）
        double angle_deg = angle_rad * 180 / CV_PI;  // 将角度从弧度转换为度

        if (angle_deg >= min_angle && angle_deg <= max_angle) {
            filtered_lines.push_back(line);
        }
    }
    // 在图像上绘制筛选后的线条
    for (size_t i = 0; i < filtered_lines.size(); i++)
    {
        const cv::Vec4i& line = filtered_lines[i];
        const cv::Point pt1(line[0], line[1]);
        const cv::Point pt2(line[2], line[3]);
        cv::line(line_image, pt1, pt2, cv::Scalar(255), 2, cv::LINE_AA);
    }
    //再画右线
    min_angle = 18;  // 最小角度（以度为单位）
    max_angle = 90;  // 最大角度（以度为单位）
    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i line = lines[i];
        double angle_rad = atan2(line[3] - line[1], line[2] - line[0]);  // 计算角度（弧度）
        double angle_deg = angle_rad * 180 / CV_PI;  // 将角度从弧度转换为度

        if (angle_deg >= min_angle && angle_deg <= max_angle)
        {
            filtered_lines.push_back(line);
        }
    }
    // 在图像上绘制筛选后的线条
    for (size_t i = 0; i < filtered_lines.size(); i++)
    {
        const cv::Vec4i& line = filtered_lines[i];
        const cv::Point pt1(line[0], line[1]);
        const cv::Point pt2(line[2], line[3]);
        cv::line(line_image, pt1, pt2, cv::Scalar(255), 2, cv::LINE_AA);
    }
    // 最后对图像进行膨胀处理
    // 膨胀核
    cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    // 对边缘检测后的图像进行膨胀处理
    cv::Mat dilated_ca2;
    cv::dilate(line_image, dilated_ca2, kernel2, cv::Point(-1, -1), 1);
    // 过滤指定范围外的角度=============《《《结束》》
    //=========================================>>>>>>>>>>>
    // 扫线
   int car_error =  Image_Handle22(dilated_ca2, hsv_image, dilated_ca);
    //测试
    //cv::imshow("data", gaussian_blur);
    //if(CA_Imageflag == 1)   
   // cv::imshow("ca", ca);
   if(saidao == 1)
   { cv::imshow("dilated_ca2", dilated_ca2);
 cv::imshow("膨胀后的ca", dilated_ca);}
    if(saidao1 == 1)
   { cv::imshow("膨胀后的ca", dilated_ca);}
    if(saidao == 3)
   { cv::imshow("膨胀后的ca", dilated_ca);}


   // 打印 cropped_image 的尺寸
//std::cout << "cropped_image 尺寸: " << cropped_image.size() << std::endl;
// 打印 dilated_ca2 的尺寸
//std::cout << "dilated_ca2 尺寸: " << dilated_ca2.size() << std::endl;
    // cv::imshow("dilated_ca2", dilated_ca2);//裁剪后的原图
    // cv::imshow("膨胀后的ca", dilated_ca);
    // std::cout << "图像尺寸: " << dilated_ca2.size() << std::endl;
    return car_error;
}

char element_flag = 26;//元素标志
char Change_track_flag = 1;//换道标志
int flag_stop_zhuangpaizi = 0;

//扫线
int bizhanground=0;
int bizhangenable=1;//默认0为不允许，1为允许
int yellowenable=0;
int BZget;
int stopbanma=0;
// error突变保护相关变量
int prev_error = 0;          // 上一帧的error值
bool error_freeze_flag = false;  // 冻结标志（true：处于冻结状态）
int frozen_error_val = 25;    // 冻结时保持的error值（默认25左右）

/**
 * 处理error值突变：当error从25左右突变为负数时，冻结在25附近，直到error回到10左右再释放
 * @param current_error 当前计算的error值
 * @return 处理后的稳定error值
 */
int stabilize_error(int current_error) {
    // 1. 判断是否触发冻结条件：上一帧error≥20（接近25），当前帧突然≤0（负数）
    if (!error_freeze_flag && prev_error >= 6 && current_error <= 0) {
        error_freeze_flag = true;  // 触发冻结
        frozen_error_val = prev_error;  // 冻结在上一帧的25左右
        std::cout << "触发error冻结！冻结值：" << frozen_error_val << std::endl;
        return frozen_error_val;
    }

    // 2. 若已冻结，判断是否解除冻结：当前error≥10（回到10左右）
    if (error_freeze_flag) {
        if (current_error >= 0) {
            error_freeze_flag = false;  // 解除冻结
            std::cout << "解除error冻结！当前值：" << current_error << std::endl;
            prev_error = current_error;  // 更新历史值
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



int Image_Handle22(cv::Mat data,cv::Mat YUANTU, cv::Mat BANMA)  //图像320 *120
{
    static int BZ_con[9];
    int errroer_car = 0;
    int16 i;                                // 控制行
    int16 j;                                // 用于二次循环
    Line_Count = 0;        // 赛道行数复位
    Left_Add_Start = 0;        // 复位左补线起始行坐标
    Right_Add_Start = 0;        // 复位右补线起始行坐标
    Left_Add_Stop = 0;        // 复位左补线起终止坐标
    Right_Add_Stop = 0;        // 复位右补线起终止坐标
    for (i = ROW-1; i >= 9; i -= 2)//赛道初始化
    {
        Left_Add_Flag[i] = 1;
        Right_Add_Flag[i] = 1;
    }

    // std::cout << "===== 所有行的left_Add_Line值 =====" << std::endl;
    // for (int k = ROW-1; k >= 9; k -= 2) {
    //     std::cout << "行" << k << ": " << Left_Add_Line[k] << std::endl;
    // }
    // std::cout << "===== 所有行的Right_Add_Line值 =====" << std::endl;
    //     for (int k = ROW-1; k >= 9; k -= 2) {
    //     std::cout << "行" << k << ": " << Right_Add_Line[k] << std::endl;
    // }

    /***************************** 第一行特殊处理 **************************/
    int y = First_Line_Handle(data);//虚拟首行中点
    /*处理普通赛道开始*/
    for (i = ROW-1; i >= 9; i -= 2)                  // 仅处理前40行图像，隔行后仅处理20行数据
    {
        Line_Count = i;
        Earge_Search_Mid(i, data, Mid_Line[i + 2], 1, COL - 1, Left_Line, Right_Line, Left_Add_Line, Right_Add_Line, 0);//搜寻并保存边界数据
    }
    LinearInterpolation();//中线线性插值
    // ---------------------------------------------
    if (XUNJI_Imageflag == 1)//图像显示
    {
        for (i = ROW-1; i >= 9; i -= 2)
        {
            cv::Point pa(Right_Add_Line[i] - 5, i);//记录点
            cv::Point pb(Left_Add_Line[i] + 5, i);//记录点
            cv::Point pc(Interpolated_Liness[i], i);//记录点
            //描点画线
            cv::circle(data, pa, 5, 255);
            cv::circle(data, pb, 8, 255);
            cv::circle(data, pc, 1, 255);
        }



            // 压缩图像尺寸让霍夫检测跑快点
        cv::Mat kjkjkj;
        cv::resize(data, kjkjkj, cv::Size(), 0.5, 0.5);
        cv::imshow("循迹", kjkjkj);//最终扫线
    }
if(yellowenable==1)
{
    yellow_chuli(YUANTU);
        std::cout << "CAR_STOP_FLAG 的值为: " << CAR_STOP_FLAG << std::endl;

}

//////////////////////////避障处理部分//////////////////////////////
if(bizhangenable==1)
{           int BZ_GET = 0;
            BZ_con[8] = BZ_chuli(YUANTU);
            BZ_con[0] = BZ_con[1];
   	        BZ_con[1] = BZ_con[2];
  	        BZ_con[2] = BZ_con[3];
    		BZ_con[3] = BZ_con[4];
    		BZ_con[4] = BZ_con[5];
    		BZ_con[5] = BZ_con[6];
    		BZ_con[6] = BZ_con[7];
    		BZ_con[7] = BZ_con[8];
    		
   
          for (int i = 0;i <= 7;i++)
    		{
        		if (BZ_con[i] == 1)
        		{
            		BZ_GET++;
        		}
    		}
             if(BZ_GET >= 2)//避障
    		{
        		 BZget = BZ_PANDUAN_2();//判断障碍是否在赛道内
        		 BS_BZ_FLAG = BZget;
        		if (BZget == 1)//显示一下
        		{
        	  		
            		cv::Point BZ(find_XYdata[0], find_XYdata[1]);//记录点
            		cv::circle(data, BZ, 10, 255);//画出障碍物位置
                    std::cout << "有障碍物在赛道内" << std::endl;
                    std::cout << "find_XYdata[0]: " << find_XYdata[0] << std::endl;
                    std::cout << "find_XYdata[1]: " << find_XYdata[1] << std::endl;
            		BZ_LuoJISET();//避障逻辑
                    bizhanground++;
                    std::cout << "允许斑马线触发" << std::endl;
        		}
        		else std::cout << "有障碍物在赛道外" << std::endl;
        
    		}
     }

            if(banmaenable==1)
            {
             // BMGet = BanMa_Find(banmachuli);
               BMGet=BanMa_Find111(BANMA);
             // BanMa_Find111(BANMA);
              std::cout << "BMGet " << BMGet << std::endl;

            }
            // BMGet = BanMa_Find(banmachuli);
            // BanMa_Find111(BANMA);
            // std::cout << "BMGet " << BMGet << std::endl;
            if(BMGet==1&&stopbanma==0)
            {
                stopbanma=1;
            }






    //printQ("斑马线获取 = ", BMGet);
    LinearInterpolation();//中线线性插值


    errroer_car = error_get();
    return (errroer_car );
     //int stable_error = stabilize_error(errroer_car + erroe_xiuzheng);
    //return stable_error;  // 返回处理后的稳定值
}








uint8 Weight_th[110] =
{
     2,  2,  2,  2,  2,  3,  3,  5,5,  5,
    5,  5,  5,  5,  5,  5,  5,5,5, 5,
    4,  4,  4,  4,  4,  4,  4,  4,4,  4,
     3, 3,  3,  3,  3,  3,  3,  3, 3,  3,
    7, 7,  7,  7,  7,  7,  8,  8, 8,  9,
      8, 8,  8,  8,  8,  8,  8,  8, 8,  9,
     5,  5,  5,  5,  5,  5,  6,  6,6,  6,
    2,  2,  2,  2,  2,  2,  2,  2,2,  2,
    1,  1,  1,  1,  1,  1,  1,  1,1,  1,
     1,  1,  1,  1,  1,  1,  1,  1,1,  1, 
};     //加权平均参数
int error_get(void)
{
    long error_in = 0;
    int error_out = 0;
    long Weight_Count = 0;
    int j = 0;
    for (int i = ROW-1; i >= 9 ; i -= 2) 
    {
        error_in += (Mid_Line[i] - 320) * Weight_th[j];
        Weight_Count += Weight_th[j];
        j++;
    }
    error_out = error_in / Weight_Count;
    if (error_out < -160)
    {
        error_out = -160;
    }
    if (error_out > 160)  
    {
        error_out = 160;
    }
    return error_out/4;
}


int BZ_Imageflag = 0, BM_Imageflag = 0,yellow_Imageflag = 0,blue_Imageflag = 0,XUNJI_Imageflag = 0,CA_Imageflag = 0;





///////////////////避障部分//////////////////////



//



int find_XYdata[3];//障碍物坐标
int find_XYdata_second[3];//障碍物坐标


void bizhangBuxian(int data_X, int data_Y, int bizhang_fangxiang)//0 向左避障  1向右避障备份

{
    if (bizhang_fangxiang == 0)//左避障
    {
        for (int i = ROW-1; i >= 9; i -= 2)
        {
        	  	  Right_Add_Line[i] = data_X - Bizhang_line_move-6;//lab
        }
        std::cout << "向左" << std::endl;
    }
    else  if (bizhang_fangxiang == 1)//右避障
    {
        for (int i = ROW-1; i >= 9; i -= 2)
        {
                Left_Add_Line[i] = data_X + Bizhang_line_move + 6;
        } 
        std::cout << "向右" << std::endl;
    }
}
cv::Mat red_mask;
void BZ_LuoJISET(void)//避障逻辑
{
    static int zhangai_flag1 = 0;
    static int zhangai_flag22 = 1;
    static int zhangai_Left_or_Right = 0;
    static int nums = 0;
    std::cout << "find_XYdata[1] 的值为: " << find_XYdata[1] << std::endl;
    if (find_XYdata[1] > 40 && zhangai_flag1 == 0)//在图像下方出现
    {
       nums++;
       if(nums >= 1)//3帧
        {
            nums = 0;
            zhangai_flag1 = 1;
        }
        
    }
    if (zhangai_flag1 == 1)//又出现在上方
    {
        if (find_XYdata[1] <= 30)
        {
            std::cout << "down" << std::endl;
           nums++;
            if (nums >= 1)//3帧
            {
                nums = 0;
                zhangai_flag1 = 2;
            }
        }
    }
    if (zhangai_flag1 == 2)//切换
    {
        zhangai_flag22++;
        if (zhangai_flag22 >= 2) zhangai_flag22 = 0;
        zhangai_flag1 = 0;
    }
      if (zhangai_flag22 == 0) { std::cout << "左避障" << std::endl; zhangai_Left_or_Right = 0; }
      if (zhangai_flag22 == 1) { std::cout << "右避障" << std::endl; zhangai_Left_or_Right = 1; }
    //--------------
    bizhangBuxian(find_XYdata[0], find_XYdata[1], zhangai_Left_or_Right);//避障改线
    Mid_Line_Repair();
}

int red1max=0,red2max=0,red3max=0,red1min=0,red2min=0,red3min=0;
int BZ_chuli(cv::Mat BZdata)//传入原图
{
  	      cv::Mat hsvvvv;
          cv::cvtColor(cropped_imageddddd, hsvvvv, cv::COLOR_BGR2HSV);
//        cv::imshow("yuantu2", BZdata);
 //cv::imshow("yuantu", cropped_imageddddd);
    // 定义红色和蓝色范围
//红色阈值1

    cv::Scalar red_lower1(143,100,89);
    cv::Scalar red_upper1(179, 255, 255);

//     cv::Scalar blue_lower(Hmin ,Smin, Vmin);
//    cv::Scalar blue_upper(Hmax, Smax, Vmax);

//红色阈值2
 //   cv::Scalar red_lower2(0, 140, 150);
  //  cv::Scalar red_upper2(10, 255, 250);
//蓝色阈值
      // cv::Scalar blue_lower(144,154, 154);
      // cv::Scalar blue_upper(180, 255, 255);//高强光
    // cv::Scalar blue_lower(144,54, 117);
    // cv::Scalar blue_upper(180, 255, 255);//强光

    // cv::Scalar blue_lower(115,115, 148);
    // cv::Scalar blue_upper(180, 255, 255);//晚上

    cv::Mat  blue_mask,red_mask2; //
    //黄色图像
    cv::Mat yellow_mask;


if(red_set ==0)
{
// red1max=179;
// red2max=225;
// red3max=225;
// red1min=152;
// red2min=61;
// red3min=59;
//     cv::Scalar blue_lower(red1min,red2min, red3min);
//     cv::Scalar blue_upper(red1max, red2max, red3max);//晚上 
    cv::Scalar blue_lower(136,150, 49);
    cv::Scalar blue_upper(179, 255, 255);//晚上
        cv::inRange(hsvvvv, blue_lower, blue_upper, blue_mask);
}
    
//    cv::Scalar blue_lower(144,154, 154);
//    cv::Scalar blue_upper(180, 255, 255);//高强光
//    cv::Scalar blue_lower(144,54, 117);
//    cv::Scalar blue_upper(180, 255, 255);//强光

   if(red_set == 1)
{
   cv::Scalar blue_lower(Hmin ,Smin, Vmin);
   cv::Scalar blue_upper(Hmax, Smax, Vmax);
    cv::inRange(hsvvvv, blue_lower, blue_upper, blue_mask);
}
   // cv::Scalar blue_lower(120,70, 70);
    // cv::Scalar blue_upper(180, 255, 255);//晚上11/27
///黄色范围

    if(yellow_set == 0)

{
    // cv::Scalar yellow_lower(7, 10, 140);
    // cv::Scalar yellow_upper(37, 255, 255);
    cv::Scalar yellow_lower(136,150, 49);
    cv::Scalar yellow_upper(179, 255, 255);//晚上
    cv::inRange(BZdata, yellow_lower, yellow_upper, yellow_mask);
}
    if(yellow_set == 1)
{
    cv::Scalar yellow_lower(Hmin ,Smin, Vmin);
    cv::Scalar yellow_upper(Hmax, Smax, Vmax);   
    cv::inRange(BZdata, yellow_lower, yellow_upper, yellow_mask); 

}


    // 颜色提取（红色和蓝色）

    

    cv::inRange(BZdata, red_lower1, red_upper1, red_mask);


  
    //腐蚀黄色图像------------------------------
     cv::Mat kerneyellow = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1)); // 可以调整核的大小 
     cv::erode(yellow_mask, yellow_mask, kerneyellow, cv::Point(-1, -1), 1); 
     // //膨胀黄色图像
     cv::Mat kerneyellow22 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1)); // 可以调整核的大小 
     cv::dilate(yellow_mask, yellow_mask, kerneyellow22, cv::Point(-1, -1), 1);
	 //CAR_STOP_FLAG =  CAR_STOP(yellow_mask);//看到黄色后停车  要改一下找点的范围，不然会越界---------------------------------------------------------
    //cv::imshow("yellow_mask", yellow_mask);
    // //腐蚀蓝色和红色
     cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1)); // 可以调整核的大小  .
    //  cv::erode(blue_mask, blue_mask, kernel3, cv::Point(-1, -1), 1); 
    //  cv::erode(blue_mask, blue_mask, kernel3, cv::Point(-1, -1), 1);    
    // // 膨胀蓝色和红色
     cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1,1));
     cv::dilate(blue_mask, blue_mask, kernel2, cv::Point(-1, -1), 1);
     cv::dilate(blue_mask, blue_mask, kernel2, cv::Point(-1, -1), 1);
   //  cv::imshow("BZdata", blue_mask);
     if(red_set == 1) 
     {  
         cv::imshow("BZdata", blue_mask);

     }
    if(yellow_set == 1) 
     {  
         cv::imshow("BZdata", yellow_mask);

     }
      cv::waitKey(10);
 
    //
	   if(BZ_Imageflag == 1)      cv::imshow("red_mask", red_mask);
       
       
    // if(yellow_Imageflag == 1)  cv::imshow("yellow_mask", yellow_mask); 
    // if(blue_Imageflag == 1)    cv::imshow("blue_mask", blue_mask);
    
    
    
    //    if(TIME_BIZHANG == 0)  return 0;//跳过避障
		// 寻找蓝色色块的轮廓
   
		std::vector<std::vector<cv::Point>> red_contours;
		cv::findContours(blue_mask, red_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		// 找到最大的红色色块
		double max_area = 0;
		int max_area_index = -1;
		for (int i = 0; i < red_contours.size(); i++)
		{
  			  double area = cv::contourArea(red_contours[i]);
   			  if (area > max_area)
	   		  {
     		     max_area = area;
      		     max_area_index = i;
    		       }
	     }
         if(BZ_Imageflag == 2)  cv::imshow("标记红色色块", BZdata);
		// 标记最大的红色色块并打印中心点坐标
		if (max_area_index >= 0) 
		{
   		 cv::drawContours(BZdata, red_contours, max_area_index, cv::Scalar(0, 0, 255), 2);
   		 // 获取最大红色色块的边界框
   		 cv::Rect bounding_rect = cv::boundingRect(red_contours[max_area_index]);
   		 // 计算最大红色色块的中心点坐标
   		 int center_x = (bounding_rect.x + bounding_rect.width / 2)/2;
   		 int center_y = (bounding_rect.y + bounding_rect.height / 2)/2;
   		 // 打印中心点坐标
   		 //std::cout << "最大红色色块的中心点坐标: (" << center_x << ", " << center_y << ")" <<"像素点数目"<<max_area<< std::endl;
   		 if(max_area <= 2)   return 0;//障碍物大小
   		 find_XYdata[0] = center_x;
           find_XYdata[1] = center_y;
    	//cv::imshow("标记红色色块", BZdata);
		//std::cout << "center_x的值为: " << center_x << std::endl;
        //std::cout << "center_y的值为: " << center_y << std::endl;
   		//std::cout << "find_XYdata[0]: " << find_XYdata[0] << std::endl;
        //std::cout << "find_XYdata[1]: " << find_XYdata[1] << std::endl;
        
           return 1;
		}
       
	     // 展示标记后的图像

    return 0;
}
int yellow_chuli(cv::Mat BZdata)//传入原图
{
  	cv::Mat hsvvvv;
    cv::cvtColor(cropped_imageddddd, hsvvvv, cv::COLOR_BGR2HSV);

///黄色范围
    // cv::Scalar yellow_lower(7, 10, 140);
    // cv::Scalar yellow_upper(37, 255, 255);
    cv::Scalar yellow_lower(142,45, 55);
    cv::Scalar yellow_upper(179, 255, 255);//晚上

    // cv::Scalar yellow_lower(7, 17, 80);
    // cv::Scalar yellow_upper(43, 255, 227);

    //黄色图像
    cv::Mat yellow_mask;
    cv::inRange(BZdata, yellow_lower, yellow_upper, yellow_mask);


  
    //腐蚀黄色图像------------------------------
     cv::Mat kerneyellow = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1)); // 可以调整核的大小 
     cv::erode(yellow_mask, yellow_mask, kerneyellow, cv::Point(-1, -1), 1); 
     // //膨胀黄色图像
     cv::Mat kerneyellow22 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1)); // 可以调整核的大小 
     cv::dilate(yellow_mask, yellow_mask, kerneyellow22, cv::Point(-1, -1), 1);
	 CAR_STOP_FLAG =  CAR_STOP(yellow_mask);//看到黄色后停车  要改一下找点的范围，不然会越界---------------------------------------------------------
     //cv::imshow("yellow_mask", yellow_mask);

    // cv::imshow("BZdata", yellow_mask);
 if(yellow_set == 1)
{
    cv::imshow("BZdata", yellow_mask);
      cv::waitKey(10);
}

 

       
	     // 展示标记后的图像

    return 0;
}










int BZ_PANDUAN_2(void)//判断障碍物是否在赛道内
{
    int Y = 0;
    if(find_XYdata[1] <=10) return 0;
    for (int i = ROW-1; i >= 9; i -= 2) 
    {
        if (find_XYdata[1] >= i)//找到障碍点所在行
        {
        		 Y = i;
                break;
        }
        if (i <= 11) return 0;//不认为在
    }
    if (find_XYdata[0] <= Left_Add_Line[Y] -25|| find_XYdata[0] >= Right_Add_Line[Y] + 25 )//判断是否在赛道内
    {
        return 0;
    }
    else return 1;
}




////////////////////////////
//---------斑马线---------//
////////////////////////////
int BanMa_Find111(cv::Mat BanMa_Find_data)
{
    
    std::vector<std::vector<cv::Point>> contours;// 查找图像中的轮廓
    cv::findContours(BanMa_Find_data, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    cv::Mat contour_img = BanMa_Find_data.clone();// 创建一个副本以便绘制轮廓
    static int count_BMXduilie[8];
    int Y_point[20];//避障坐标值
    int count_BMX = 0;//斑马线标志
    // 定义矩形的大小宽度阈值（根据实际需要调整）
    int min_wh = 5;    // 最小宽度
    int max_wh = 55;   // 最大宽度
    // 遍历每个找到的轮廓
    for (const auto& contour : contours)
    {
        cv::Rect rect = cv::boundingRect(contour);
        if (min_wh <= rect.height && rect.height < max_wh && min_wh <= rect.width && rect.width < max_wh)
        {
            //过滤赛道外的轮廓
            if (rect.y >= 10 && rect.y <= 85)
            {
                if (rect.y % 2 == 0) rect.y = rect.y - 1;
                if (rect.y >= 85)
                {
                    if(rect.x >= (20) && rect.x <= (300))
                    {
                        cv::rectangle(contour_img, rect, cv::Scalar(255), 2);
                        // 打印轮廓坐标信息
                       // std::cout << "轮廓坐标X =  " << rect.x << " 轮廓坐标Y =  " << rect.y << std::endl;
                        count_BMX++;
				                  Y_point[count_BMX] = rect.y;
                    }
                }
                else
                {
                    if (rect.x >= (Left_Add_Line[rect.y] - 20) && rect.x <= (Right_Add_Line[rect.y] + 20))
                    {
                        cv::rectangle(contour_img, rect, cv::Scalar(255), 2);
                        // 打印轮廓坐标信息
                       // std::cout << "轮廓坐标X =  " << rect.x << " 轮廓坐标Y =  " << rect.y << std::endl;
                        count_BMX++;
                        Y_point[count_BMX] = rect.y;
                    }
                }
            }
            if(count_BMX >= 6) count_BMX = 6;
        }
    }
    int pingJun = (Y_point[1] + Y_point[2] + Y_point[3]+ Y_point[4])/4;
    pingJun = (Q_jdz(pingJun - Y_point[1]) + Q_jdz(pingJun - Y_point[2]) + Q_jdz(pingJun - Y_point[3])+ Q_jdz(pingJun - Y_point[4]))/4;
    //printQ("pingJun",pingJun);
    if (count_BMX >= 4)
    {
    	   banmaxian_Y = (Y_point[1] + Y_point[2] +Y_point[3] +Y_point[4])/4;
    	 //  std::cout << "斑马线坐标  =  " << banmaxian_Y << std::endl;        //////
        count_BMX = 0;
        count_BMXduilie[7] = 1;
         if(pingJun >= 15)//看他们横坐标的平均值---标记------------------------------------------------修改此处可以调整斑马线停车距离
    		{
    			return 0;
   		}
    }
    else
    {
        count_BMX = 0;
        count_BMXduilie[7] = 0;
    }

    count_BMXduilie[0] = count_BMXduilie[1];
    count_BMXduilie[1] = count_BMXduilie[2];
    count_BMXduilie[2] = count_BMXduilie[3];
    count_BMXduilie[3] = count_BMXduilie[4];
    count_BMXduilie[4] = count_BMXduilie[5];
    count_BMXduilie[5] = count_BMXduilie[6];
    count_BMXduilie[6] = count_BMXduilie[7];
    for (int num = 0; num <= 7; num++)
    {
        if (count_BMXduilie[num] == 1)
        {
            count_BMX++;
        }
    } 
    // 显示带有轮廓外接矩形的图像
    if(banma111 == 1)
    {cv::imshow("contour_img", contour_img);}
   // 
    //最终返回值
    if (count_BMX >= 3)  //4个就停车
    {   
        
        return 1;
    }
    else  return 0;
}

/////////////////////////
//---------------------//
/////////////////////////

int BanMa_Find(cv::Mat BanMa_Find_data) {
    // 定义感兴趣区域
    Rect roi(100, 0, 120, 240);
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
        std::cout << "pingJun " << pingJun << std::endl;
        // 如果平均偏差小于阈值，返回成功
        if (pingJun < 5) {
            std::cout << "斑马线" << std::endl;
            return 1;
        }
    }

    // 显示轮廓图像
    if(banma222 == 1)
    {cv::imshow("contour_img", contour_img);}
   // cv::imshow("contour_img", contour_img);
    return 0;
}

////////////////////////////
//---------黄线停车--------//
////////////////////////////

int CAR_STOP(cv::Mat yellow_Find_data) //320 * 96
{
    //std::cout << "图像大小: " << yellow_Find_data.size() << std::endl;
	int yellowPoint_num = 0;
	for(int i = 95;i>= 95-30;i-=1)
	{
		for(int j = 319-30;j>=30;j-=2)
		{
			if(yellow_Find_data.at<uchar>(i, j) > 100)
			{
			    yellowPoint_num++;
			}    
		}

	}
    if(false)
    {
    	    printQ("黄点数目",yellowPoint_num);
    	    cv::Point pA(10, 169);//记录点
    		cv::Point pB(10, 100);//记录点
	    //描点画线
   	     cv::circle(yellow_Find_data, pA, 8, 255);
   	 	 cv::circle(yellow_Find_data, pB, 8, 255);
   	     cv::imshow("contour_img", yellow_Find_data);//障碍物
    }
    //cv::imshow("contour_img_yy", yellow_Find_data);//障碍物
    //printQ("黄色数目",yellowPoint_num);
   // std::cout << "黄色数目: " << yellowPoint_num << std::endl;

    if(yellowPoint_num >= 80)
    {
    	    return 1;
            std::cout << "已检测到黄色线" << std::endl;
    }

    return 0;
}
/////////////////////////
//---------------------//
/////////////////////////





////////////////////////////


void onTrackbar(int, void*)
{
    // This function is called whenever a trackbar is moved.
    // You can access the current values of the trackbars here.
}
int Hmin = 0, Hmax = 0, Smin = 0, Smax = 0, Vmin = 0, Vmax = 0;
void UI_init(void)
{
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
    // createTrackbar("banmaenable ", "TrackBars", &banmaenable, 1, onTrackbar);
    // createTrackbar("bizhangenable ", "TrackBars", &bizhangenable, 1, onTrackbar);



    //------------------
    // createTrackbar("斑马�?", "TrackBars", &BM_Imageflag, 1, onTrackbar);
    // createTrackbar("红色", "TrackBars", &BZ_Imageflag, 2, onTrackbar);
    // createTrackbar("黄色", "TrackBars", &yellow_Imageflag, 1, onTrackbar); 
    // createTrackbar("蓝色", "TrackBars", &blue_Imageflag, 2, onTrackbar);
    // createTrackbar("循迹图像", "TrackBars", &XUNJI_Imageflag, 1, onTrackbar);
    // createTrackbar("边缘检�?", "TrackBars", &CA_Imageflag, 1, onTrackbar);
    // createTrackbar("代码结束", "TrackBars", &car_break, 1, onTrackbar);
    // createTrackbar("KP", "TrackBars", &UI_KP, 20, onTrackbar);
    // createTrackbar("KD", "TrackBars", &UI_KD, 50, onTrackbar);

    //createTrackbar("语音", "TrackBars", &UI_music, 1, onTrackbar);
//
//    namedWindow("page_get");
//    resizeWindow("page_get", 640, 640);
//    createTrackbar("hmin_white", "page_get", &hmin_white, 179, onTrackbar);
//    createTrackbar("smin_white", "page_get", &smin_white, 254, onTrackbar);
//    createTrackbar("vmin_white", "page_get", &vmin_white, 254, onTrackbar);
//    createTrackbar("hmax_white", "page_get", &hmin_white, 179, onTrackbar);
//    createTrackbar("smax_white", "page_get", &smin_white, 254, onTrackbar);
//    createTrackbar("vmax_white", "page_get", &vmin_white, 254, onTrackbar);
//    createTrackbar("White_Imageflag", "page_get", &White_Imageflag,4, onTrackbar);
//    createTrackbar("thre", "page_get", &thre, 5000, onTrackbar);
}

