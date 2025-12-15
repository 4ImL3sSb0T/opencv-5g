#include <iostream>
#include <opencv2/opencv.hpp>
extern int ABenable;//AB判断允许（1为允许）
extern int LRenable;//LR判断允许（1为允许）
extern int panduanenable;//总识别判断允许（1为允许）
extern int ABvual;//AB判断的值
extern int LRvual;//左右箭头判断的值
extern int Aflage;//识别为A的次数
extern int Lflage;//识别为L的次数
extern int Bflage;//识别为B的次数
extern int Rflage;//识别为R的次数
extern int ABround;//AB轮次
extern int LRround;//LR轮次
extern int ABoutput;//AB判断最终输出
extern int LRoutput;//LR判断最终输出
extern int AB(const cv::Mat& frame1);
extern int arrow(const cv::Mat& input_frame);

void panduan(void);
//