//==========================头文件区==========================//
#include "A4PaperExtractor.hpp"
#include "ArrowDetector.hpp"
#include "init.hpp"
#include "shibie.hpp"
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc_c.h>
#include <opencv4/opencv2/opencv.hpp>
#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <yolo_detector.hpp>


extern "C" {
#include "serial.h"
}

#include "image_Q.hpp"
#include <thread>

//==========================头文件区==========================//
int speed = -1200;
//==========================定义变量区==========================//

int errchange1 = 0;
int tiaosu = 1200;
int fangxiang = 1;
int direction = 0; // 箭头方向
int jiantoucount = 0;
int directionclock = 0;
int yindaoquBZ = 0;
int car_error;
int time_flagggggg = 0;
int cone_result = 0;   // 锥桶引导区检测结果
int yindaoqustart = 0; // 引导区开始标志位
int detectConestart = 0;
int LRoutput2 = 0; // 换道输出  1右  2左
int SwitchLaneover = 0;
int leftrightflag = 0;
int rukustart = 0;
int banmaclock =0;
int Leftflage = 0;
int Rightflage = 0;
int LRoutput114 = 0; 
int bizhangtime = 9999;
int bizhangover = 0;

const int pwm_pin = 13;
const float pwm_range = 40000.0;
const float pwm_frequency = 200.0;
const float pwm_duty_cycle_unlock = 10000.0;
const int pwm_pinser = 12;


// ========== YOLO AB检测相关变量 ==========
int yoloABResult = 0;      // YOLOAB检测结果: 0=未检测, 1=检测到A, 2=检测到B
int yoloABDetectEnable = 0;  // YOLOAB检测使能标志
std::string yoloABModelPath = "../model/ab_pro.onnx";  // 模型路径，根据实际修改
int Aflag=0;
int Bflag=0;
int ABcount=0;
int yoloABOver = 0; // 1检测结束

// ========== YOLO 箭头检测相关变量 ==========
int yoloArrowResult = 0;      // YOLO箭头检测结果: 0=未检测, 1=检测到左箭头, 2=检测到右箭头
int yoloArrowDetectEnable = 0;  // YOLO箭头检测使能标志
std::string yoloArrowModelPath = "../model/LRpromax.onnx";  // 箭头模型路径，根据实际修改
int Lflag=0;
int Rflag=0;
int LRcount=0;

// ========================================================//

#define MOTOR_PIN 13
#define SERVO_PIN 12
#define YUNTAI_X_PIN 22 // 云台左右
#define YUNTAI_Y_PIN 23 // 云台上下
#define PWM_BASE 10000	// 电机PWM基准值
#define PWM_RANGE 40000.0
#define PWM_FREQ 200.0
#define SERVO_MID 65 // 舵机中位值

//==========================定义变量区==========================//

//=============================================================//

void motorSet(int data) //==========================电机驱动函数==========================//

{
	if (data > 8000)
		data = 8000;
	if (data < -8000)
		data = -8000;
	if (data < 0) // 反转
	{
		data = -data;
		gpioPWM(13, data + 10000);
	} else if (data >= 0) {
		gpioPWM(13, 10000 - data);
	}
}

void servor_set(int data) // 舵机值更改  +-18//65
{
	int max_offset = 15;
	if (banmaenable == 1)
		max_offset = 4;
	if (data >= SERVO_MID + max_offset) {
		data = SERVO_MID + max_offset;
	}
	if (data <= SERVO_MID - 15) {
		data = SERVO_MID - 15;
	}
	gpioPWM(12, data);
}

void pwnInit(void) //==========================pwm初始化函数==========================//

{
	//    //-----
	if (gpioInitialise() < 0) //==============如果gpioInitialise<0则输出gpio  wrong
	{
		std::cout << "GPIO wrong in pwm wrong" << std::endl;
		return;
	} else
		std::cout << "GPIO PWM ok" << std::endl; //==============如果正常则输出gpio  wrong
	//--------------
	gpioSetMode(pwm_pinser, PI_OUTPUT);
	gpioSetPWMfrequency(pwm_pinser, 50);
	gpioSetPWMrange(pwm_pinser, 1000);
	gpioPWM(pwm_pinser, 65);

	//----------------------------
	if (gpioInitialise() < 0) {
		std::cout << "GPIO wrong in pwm wrong" << std::endl;
		return;
	}
	gpioSetMode(pwm_pin, PI_OUTPUT);
	gpioSetPWMfrequency(pwm_pin, pwm_frequency);
	gpioSetPWMrange(pwm_pin, pwm_range);
	gpioPWM(pwm_pin, pwm_duty_cycle_unlock);
	time_sleep(1);
	//-----------------------------------
}

void yuntaiInit(void) //==========================云台初始化函数==========================//
{
	//    //-----
	if (gpioInitialise() < 0) //---第一个
	{
		std::cout << "GPIO wrong in yuntaiinit" << std::endl;
		return;
	} else
		std::cout << "GPIO yuntai ok" << std::endl;
	//--------------
	gpioSetMode(22, PI_OUTPUT);
	gpioSetPWMfrequency(22, 50);
	gpioSetPWMrange(22, 1000);
	gpioPWM(22, 71); // 大左小右(云台左右)
	//---------
	//    //-----
	if (gpioInitialise() < 0) // 第二个
	{
		std::cout << "GPIO wrong in yuntaiinit" << std::endl;
		return;
	} else
		std::cout << "GPIO  yuntai ok" << std::endl;
	//--------------
	gpioSetMode(23, PI_OUTPUT);
	gpioSetPWMfrequency(23, 50);
	gpioSetPWMrange(23, 1000);
	gpioPWM(23, 71); // 大上下小（云台上下）
}

void pidcont(int Error) // 控制
{
	static int last_Car_Out = 0;
	static int last_error = 0;
	float Car_P = 0; // 0.8
	float Car_D = 0;
	float gkd = 0;
	int Car_Out = 0;
	int mod = 21;
	Error = Error - errchange1 - 7; // 减小向左跑
	if (mod == 2) {
		Car_P = 0.2;
		Car_D = 0.3; // 5500
	}

	if (tiaosu == 4000) // 4000速度  初始速度
	{
		Car_P = 0.15;
		Car_D = 0.34;
//		errchange1 = 8;
	}
	if (tiaosu == 4500) // 4000速度  初始速度
	{
		Car_P = 0.20;
		Car_D = 0.3;
		errchange1 = 8;
	}
	if (tiaosu == 5000) // 4000速度  初始速度
	{
		Car_P = 0.18;
		Car_D = 0.3;
		errchange1 = 9;
	}
	if (tiaosu == 2000) // 2000速度
	{
		Car_P = 0.25;
		Car_D = 0.25;
	}
	if (tiaosu == 2500) // 2500/2700速度
	{
		Car_P = 0.2;
		Car_D = 0.3;
//		errchange1 = -7;	
		if(banmaenable == 1)	
		{
			// Car_P = 0.2;
			// Car_D = 0.15;
			// errchange1 = -5;//倒数第二
			Car_P = 0.11;
			Car_D = 0.2;
			errchange1 = -4;
		}
	}
	if (tiaosu == 3000) // 3000速度
	{
		Car_P = 0.3;
		Car_D = 0.3;
		errchange1 = -7;
	}
	if (tiaosu == 3700) // 3700速度
	{
		Car_P = 0.59;
		Car_D = 0.9;
	}
	if (tiaosu == 3500) // 3500速度
	{
		// Car_P = 0.15;
		// Car_D = 0.7;
		errchange1 = -7;
		Car_P = 0.13;
		Car_D = 0.4;
	}
	if (tiaosu == 1500) // 1500速度
	{
		Car_P = 0.2;
		Car_D = 0.3;
		errchange1 = -5;
	}
	if (tiaosu == 1000) // 1000速度
	{
		Car_P = 0.2;
		Car_D = 0.2;
	}
	if (tiaosu == 1200) // 1000速度
	{
		Car_P = 0.25;
		Car_D = 0.4;
	}

	Car_Out = -(Car_P * Error + Car_D * (Error - last_error));

	if (Car_Out < -15)
		Car_Out = -15;
	if (Car_Out > 15)
		Car_Out = 15;
	servor_set(Car_Out + 65); // 舵机值  //正左负右
	last_error = Error;
	last_Car_Out = Car_Out;
	//==========================================================
}

// int hdflage = 0;//换道标志位
int hdscan1 = 1;
int hdscan2 = 1;
int h1 = 0;
int h2 = 0;
int hdlogic(void) {
	// if (car_error <= -20) {
	//   h1 = 1;
	// }
	// if (car_error > 0 && h1 == 1) {
	//   hdscan1 = 2;
	// }

	// if (car_error >= 20) {
	//   h2 = 1;
	// }
	// if (car_error < 0 && h2 == 1) {
	//   hdscan2 = 2;
	// }
	return 0;
}
int huandao(void) {
	if (hdflage == 1) // 当换道标记为1时，允许换道
	{
		std::cout << "开始执行换道相关操作" << std::endl;

		hdscan1 = 1;
		hdscan2 = 1;
		if (LRoutput == 1) // 执行右换道
		{
			hdlogic();
			if (hdscan1 == 1) {
				servor_set(50); // 舵机打角
				fangxiang = 2;	// 禁止pid控制方向

				// speed = -1200;  // 强制给一个前进速度
				// tiaosu = 1200;
				motorSet(-1200);
				time_sleep(1.85);
				hdscan1 = 2;
				std::cout << "开始换道右---" << std::endl;
			}
			if (hdscan1 == 2) {
				servor_set(70); // 舵机打角
				time_sleep(0.3);
				fangxiang = 1; // 允许pid控制
				std::cout << "允许pid控制" << std::endl;
				LRoutput = 0; // 置零
				speed =
					-1500; //--------------------------------------------------------------此处调整变道后面的航速
				tiaosu = 1500;
				hdflage = 2;
				bizhangenable = 0; // 不允许避障
				yindaoqustart = 1; // 开始引导区
			}
		}
		if (LRoutput == 2) // 执行左换道
		{
			hdlogic();
			if (hdscan2 == 1) {
				servor_set(80); // 舵机打角
				fangxiang = 2;	// 禁止pid控制方向
				// speed = -1200;	// 强制给一个前进速度
				// tiaosu = 1200;
				motorSet(-1200);
				time_sleep(1.55);
				hdscan2 = 2;
				std::cout << "开始换道左---" << std::endl;
			}
			if (hdscan2 == 2) {
				servor_set(60); // 舵机打角
				time_sleep(1);
				fangxiang = 1;	   // 允许pid控制
				std::cout << "允许pid控制" << std::endl;
				bizhangenable = 0; // 不允许避障
				LRoutput = 0;	   // 置零
				speed =
					-1500; //--------------------------------------------------------------此处调整变道后面的航速
				tiaosu = 1500;
				hdflage = 2;
				yindaoqustart = 1; // 开始引导区
			}
		}
		time_flagggggg = 0;
	}

	return 0;
}

int BANMATIME = 0;

int yuyinflag = 0;
// int hdflage = 0;
int times;
int stop_banmaxiankaojin = 0;
int y = 0;
int manbastart = 0;
void banmachuli(void) {

	if(manbastart == 1)
	{
	std::cout << "进入斑马线处理函数" << std::endl;
	if (stopbanma == 1) {
		times = BANMATIME;
		stopbanma = 2;
		bizhangenable = 0; // 不允许避障
	}
	if (stopbanma == 2) // 停车保持打角正常
	{
		if (stop_banmaxiankaojin == 0) {
			//  gpioPWM(13,10200);
			//  gpioPWM(13,14200);
			motorSet(4000);
			servor_set(65);
			std::this_thread::sleep_for(std::chrono::milliseconds(2500)); // 延时 1000 毫

			times = BANMATIME;
			stop_banmaxiankaojin = 1;
		}
		if (banmaxian_Y < 60) {
			speed = -1200;
			tiaosu = 1200;
			//			times = BANMATIME;
			std::cout << "times111 =" << BANMATIME << std::endl;
		}

		else
			speed = 2000;
		std::cout << "times =" << BANMATIME << std::endl;
		std::cout << "stopbanma =" << stopbanma << std::endl;
		std::cout << "banmaxian_Y =" << banmaxian_Y << std::endl;
		std::cout << "stop_banmaxiankaojin =" << stop_banmaxiankaojin << std::endl;

		if (BANMATIME - times > 1) {
			stopbanma = 200;
			speed = 0;
			y = BANMATIME;
			stop_banmaxiankaojin = 0;
			//			hdflage = 1;
//			yuyinflag = 2; // 2是允许
//			direction = 2; // 箭头检测的摄像头没改好，暂时固定右转
			yoloArrowDetectEnable = 1; // 允许箭头检测
			std::cout << "开始识别箭头" << std::endl;
		}
	}
}
}
void SwitchLane(int direction) {
	static int isOver = false;
	if (direction == 1) {
		// 向左变道
		motorSet(-1200);
		servor_set(SERVO_MID + 15);									  // 向左打满
		std::this_thread::sleep_for(std::chrono::milliseconds(1200)); // 延时
//		servor_set(SERVO_MID);										  // 回中
		servor_set(SERVO_MID -10);									  // 向左打满
		std::this_thread::sleep_for(std::chrono::milliseconds(800)); 

		cone_result = 0;
		detectConestart = 0; // 变道完成后重置引导区检测标志
		SwitchLaneover = 1;
		isOver = true;
		
	} else if (direction == 2) {
		// 向右变道
		motorSet(-1200);
		servor_set(SERVO_MID - 13);									  // 向右打满
		std::this_thread::sleep_for(std::chrono::milliseconds(1200)); // 延时
		servor_set(SERVO_MID + 10);										  // 回中
		std::this_thread::sleep_for(std::chrono::milliseconds(800)); 
		cone_result = 0;
		detectConestart = 0; // 变道完成后重置引导区检测标志
		SwitchLaneover = 1;
		isOver = true;
	}
	if (isOver) {
		static auto startTime = BANMATIME;
		if (BANMATIME - startTime > 3) {
			speed = -1500;
			tiaosu = 1500;
			isOver = false;
			yellowenable=1;
			rukustart = 1;
			banmaclock =0;
			yoloABDetectEnable=1; //开启yoloAB检测
		}
	}
}


inline int detectConeGuideTail(const cv::Mat &raw_frame,
							   const float area_threshold = 130.0f,
							   bool isShowDebugImg = false) {
	cv::Mat frame = raw_frame.clone();
	cv::resize(frame, frame, ::Size(frame.cols / 2, frame.rows / 2));
	cv::Mat line_image = cv::Mat::zeros(frame.size(), CV_8UC1);
	const bool enable_debug_visualization = isShowDebugImg;
	cv::Mat draw;
	if (enable_debug_visualization) {
		draw = frame.clone();
	}
	const int lane_y_offset = frame.rows / 3; // 仅在屏幕下 2/3 区域寻找白线（过滤天空噪声）
	const cv::Rect lane_roi_rect(0, lane_y_offset, frame.cols, frame.rows - lane_y_offset);
	if (lane_roi_rect.height <= 0) {
		std::cerr << "Invalid lane ROI" << std::endl;
		return 0;
	}
	cv::Mat lane_line_image = line_image(lane_roi_rect);

	cv::Mat hsv, mask, gray;
	cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
	cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
	cv::inRange(hsv, cv::Scalar(15, 12, 120), cv::Scalar(45, 255, 255), mask);

	// —— 根据 image_Q::TUxiang_Init 思路提取赛道白线，生成赛道边界 —— //
	cv::Mat lane_gray = gray(lane_roi_rect).clone();
	cv::Mat bilateral_blur, gaussian_blur, lane_edges;
	cv::bilateralFilter(lane_gray, bilateral_blur, 7, 60, 60);
	cv::GaussianBlur(bilateral_blur, gaussian_blur, cv::Size(5, 5), 30);
	cv::Canny(gaussian_blur, lane_edges, 30, 50);
	cv::Mat lane_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
	cv::Mat dilated_edges;
	cv::dilate(lane_edges, dilated_edges, lane_kernel, cv::Point(-1, -1), 1);

	std::vector<cv::Vec4i> all_lines;
	cv::HoughLinesP(dilated_edges, all_lines, 1, CV_PI / 180, 70, 25, 5);
	std::vector<cv::Vec4i> left_lane_segments;
	std::vector<cv::Vec4i> right_lane_segments;
	for (const auto &line : all_lines) {
		const double angle_rad = atan2(line[3] - line[1], line[2] - line[0]);
		const double angle_deg = angle_rad * 180.0 / CV_PI;
		if (angle_deg >= -90.0 && angle_deg <= -18.0) {
			left_lane_segments.push_back(line);
		} else if (angle_deg >= 18.0 && angle_deg <= 90.0) {
			right_lane_segments.push_back(line);
		}
	}

	auto drawSegments = [&](const std::vector<cv::Vec4i> &segments) {
		for (const auto &line : segments) {
			const cv::Point pt1(line[0], line[1]);
			const cv::Point pt2(line[2], line[3]);
			cv::line(lane_line_image, pt1, pt2, cv::Scalar(255), 2, cv::LINE_AA);
		}
	};
	drawSegments(left_lane_segments);
	drawSegments(right_lane_segments);

	cv::Mat lane_edges_mask = lane_line_image.clone();
	if (!lane_edges_mask.empty()) {
		cv::dilate(lane_edges_mask, lane_edges_mask,
				   cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 1);
	}
	std::vector<int> left_boundary(frame.rows, -1);
	std::vector<int> right_boundary(frame.rows, -1);
	std::vector<cv::Point> left_polyline;
	std::vector<cv::Point> right_polyline;
	const int center_x = frame.cols / 2;
	for (int y = 0; y < lane_edges_mask.rows; ++y) {
		const uchar *row_ptr = lane_edges_mask.ptr<uchar>(y);
		// 从屏幕中心向左寻找最近的白线
		for (int x = center_x; x >= 0; --x) {
			if (row_ptr[x] > 0) {
				left_boundary[y + lane_y_offset] = x;
				break;
			}
		}
		// 从屏幕中心向右寻找最近的白线
		for (int x = center_x; x < lane_edges_mask.cols; ++x) {
			if (row_ptr[x] > 0) {
				right_boundary[y + lane_y_offset] = x;
				break;
			}
		}

		const int actual_y = y + lane_y_offset;
		if (left_boundary[actual_y] >= 0 && right_boundary[actual_y] >= 0) {
			if ((right_boundary[actual_y] - left_boundary[actual_y]) < 20) {
				left_boundary[actual_y] = right_boundary[actual_y] = -1;
				continue;
			}
			left_polyline.emplace_back(left_boundary[actual_y], actual_y);
			right_polyline.emplace_back(right_boundary[actual_y], actual_y);
		}
	}

	cv::Mat track_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
	int valid_track_rows = 0;
	for (int y = 0; y < frame.rows; ++y) {
		if (left_boundary[y] < 0 || right_boundary[y] < 0)
			continue;
		++valid_track_rows;
		cv::line(track_mask, cv::Point(left_boundary[y], y), cv::Point(right_boundary[y], y),
				 cv::Scalar(255), 1);
	}
	bool has_track_mask = valid_track_rows > lane_edges_mask.rows / 4;
	if (has_track_mask) {
		cv::morphologyEx(track_mask, track_mask, cv::MORPH_CLOSE,
						 cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9)));
	} else {
		track_mask.release();
		left_polyline.clear();
		right_polyline.clear();
	}

	if (enable_debug_visualization && !track_mask.empty()) {
		cv::Mat color_mask(draw.size(), draw.type(), cv::Scalar(0, 0, 0));
		color_mask.setTo(cv::Scalar(0, 120, 255), track_mask);
		cv::addWeighted(draw, 1.0, color_mask, 0.35, 0, draw);
		if (left_polyline.size() > 1) {
			cv::polylines(draw, left_polyline, false, cv::Scalar(0, 255, 255), 2);
		}
		if (right_polyline.size() > 1) {
			cv::polylines(draw, right_polyline, false, cv::Scalar(0, 255, 255), 2);
		}
	}

	// —— 锥桶仍沿用原有轮廓判断，后续通过赛道掩膜过滤 —— //
	const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
	// cv::Mat edges_for_cone;
	// cv::Canny(gray, edges_for_cone, 30, 50);
	cv::Mat yellow_opening;
	cv::morphologyEx(mask, yellow_opening, cv::MORPH_OPEN, kernel);
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(yellow_opening.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	int count = 0;
	double roi_cone_area = 0.0;
	if (contours.size() >= 1 && !track_mask.empty()) {
		for (const auto &contour : contours) {
			const double area = cv::contourArea(contour);
			const auto rect = cv::boundingRect(contour);

			// 检查是否符合所有条件
			bool is_valid = true;

			// 检查面积是否符合阈值
			if (area < area_threshold) {
				is_valid = false;
			}

			// 限制锥桶必须位于赛道掩膜内
			if (is_valid && !track_mask.empty()) {
				const cv::Point center(rect.x + rect.width / 2, rect.y + rect.height / 2);
				if (center.x < 0 || center.x >= track_mask.cols || center.y < 0 ||
					center.y >= track_mask.rows || track_mask.at<uint8_t>(center) == 0) {
					is_valid = false;
				}
			}

			if (enable_debug_visualization) {
				// 绘制边框：绿色表示符合条件，红色表示不符合
				cv::Scalar color = is_valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
				cv::rectangle(draw, rect, color, 2);

				// 添加面积信息
				std::string area_text = "A:" + std::to_string(static_cast<int>(area));
				cv::putText(draw, area_text, cv::Point(rect.x, rect.y - 5),
							cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
			}

			if (is_valid) {
				count++;
				roi_cone_area += area;
			}
		}
	}

	if (enable_debug_visualization && !track_mask.empty()) {
		std::string roi_area_text =
			"ROI Cone Area: " + std::to_string(static_cast<int>(roi_cone_area));
		cv::putText(draw, roi_area_text, cv::Point(10, 55), cv::FONT_HERSHEY_SIMPLEX, 0.7,
					cv::Scalar(0, 200, 255), 2);
	}

	// 显示结果
	if (enable_debug_visualization) {
		std::string result_text =
			"Valid Cones: " + std::to_string(count) + "/" + std::to_string(contours.size());
		cv::putText(draw, result_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
					cv::Scalar(255, 255, 0), 2);
		cv::imshow("Cone Guide Tail Detection", draw);
		// if (!track_mask.empty()) cv::imshow("track mask", track_mask);
		cv::imshow("mask", mask);
		cv::imshow("line", line_image);
	}
	static int stable_count = 0;
	if (count >= 2)
		stable_count++;
	else
		stable_count = 0;
	if (stable_count < 4)
		return 0; // 需要连续3帧检测到锥桶才算有效
	return 1;
}



// ========== 关键配置：根据实际设备节点修改！ ==========
const int CAM1_DEVICE = 0; // 第一个摄像头设备节点（如/dev/video0）
const int CAM2_DEVICE = 2; // 第二个摄像头设备节点（如/dev/video2，根据终端输出修改）
// =====================================================

int stopflage;
int xunhuan=1;
int left_or_right=1; 
int ABoutput1=1;
int clock2;
int stopcar=1;
int clocke;
int stopflag1=0;





int run(void) {
	int debug_mode = 0;
	// 电机舵机初始化================================================================
	gpioTerminate();
	yuntaiInit(); // 云台初始化
	pwnInit();
	// 电机舵机初始化结束=========================================================

// 摄像头1、2初始化========================================================================
// 	cv::VideoCapture capture1;
// 	//	cv::VideoCapture capture2;
// 	capture1.open(0);
// 	// capture1.open(0, cv::CAP_V4L2); // 使用 V4L2 后端打开摄像头
// 	// capture1.set(cv::CAP_PROP_AUTO_EXPOSURE, 1); // 手动曝光
// 	// capture1.set(cv::CAP_PROP_EXPOSURE, 1);
// //		capture2.open(2);

// 	if (!capture1.isOpened()) {
// 		std::cout << "Can not open video capture 0" << std::endl;
// 		return -1;
// 	}



// 	// 设置编码格式为 MJPG
// 	// int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
// 	// capture1.set(cv::CAP_PROP_FOURCC, fourcc);
// 	// capture2.set(cv::CAP_PROP_FOURCC, fourcc);
// 	capture1.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
// 	capture1.set(cv::CAP_PROP_FRAME_HEIGHT, 960);
// 	// capture1.set(cv::CAP_PROP_FPS, 30);

// 	//   A4PaperExtractor a4Extractor;
// 	//   A4PaperExtractor extractor;
// 	//   extractor.setDebugMode(true);  // 开启调试模式
// 	//  ArrowDirectionDetector arrowDetector;

// 	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

// 	cv::Mat frame1;
// 	//	cv::Mat frame2;

    // 摄像头配置（树莓派优化参数）
    const int CAM_WIDTH = 320;  // 降低分辨率，减少带宽占用（优先保证稳定性）
    const int CAM_HEIGHT = 240;
    const int CAM_FPS = 15;     // 降低帧率，避免超时
    const int BUFFER_SIZE = 1;  // 最小缓冲区，减少延迟和超时概率
    A4PaperExtractor a4Extractor;
    A4PaperExtractor extractor;

	    // ========== 初始化 YOLO AB检测器 ==========
    std::cout << "Loading YOLO AB model from: " << yoloABModelPath << std::endl;
    YoloInfer::YoloDetector yoloABDetector(
        yoloABModelPath,
        320,    // model_w
        320,    // model_h
        2,      // num_classes (A和B两个类别)
        0.5f,   // conf_threshold
        0.4f    // nms_threshold
    );
    std::vector<std::string> ABlabels = {"B", "A"};  // index 0 = B, index 1 = A
    yoloABDetector.setLabels(ABlabels);
    std::cout << "YOLO AB model loaded successfully!" << std::endl;

    // ========== 初始化 YOLO 箭头检测器 ==========
    std::cout << "Loading YOLO Arrow model from: " << yoloArrowModelPath << std::endl;
    YoloInfer::YoloDetector yoloArrowDetector(
        yoloArrowModelPath,
        320,    // model_w
        320,    // model_h
        2,      // num_classes (左箭头和右箭头两个类别)
        0.5f,   // conf_threshold
        0.4f    // nms_threshold
    );
    std::vector<std::string> arrowLabels = {"B", "A"};  // index 0 = 左箭头, index 1 = 右箭头
    yoloArrowDetector.setLabels(arrowLabels);
    std::cout << "YOLO Arrow model loaded successfully!" << std::endl;




//    LetterDetector letterDetector;
//    ArrowDirectionDetector arrowDetector;
    // 初始化双摄像头（强制V4L2驱动，禁用QT后端）
    cv::VideoCapture capture1(CAM1_DEVICE, cv::CAP_V4L2);
	cv::VideoCapture capture2(CAM2_DEVICE, cv::CAP_V4L2);

    // 配置摄像头1参数（关键优化）
    if (capture1.isOpened())
    {
        capture1.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
         capture1.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
         capture1.set(cv::CAP_PROP_FRAME_HEIGHT, 960);
         //capture1.set(cv::CAP_PROP_FPS, CAM_FPS);
        capture1.set(cv::CAP_PROP_BUFFERSIZE, BUFFER_SIZE);
        capture1.set(cv::CAP_PROP_CONVERT_RGB, true);  // 强制RGB格式（解决CSI摄像头格式不兼容）
        capture1.set(cv::CAP_PROP_OPEN_TIMEOUT_MSEC, 5000);  // 打开超时5秒
        capture1.set(cv::CAP_PROP_READ_TIMEOUT_MSEC, 1000);  // 读取超时1秒
    }

    // 配置摄像头2参数（与摄像头1一致）
    if (capture2.isOpened())
    {
        capture2.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        capture2.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        capture2.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        capture2.set(cv::CAP_PROP_FPS, CAM_FPS);
        capture2.set(cv::CAP_PROP_BUFFERSIZE, BUFFER_SIZE);
        capture2.set(cv::CAP_PROP_CONVERT_RGB, true);
        capture2.set(cv::CAP_PROP_OPEN_TIMEOUT_MSEC, 5000);
        capture2.set(cv::CAP_PROP_READ_TIMEOUT_MSEC, 1000);
    }


    // 检查摄像头打开状态
    bool cam1_ok = capture1.isOpened();
    bool cam2_ok = capture2.isOpened();
    if (!cam1_ok || !cam2_ok)
    {
        std::cerr << "\n===== 摄像头打开失败 =====" << std::endl;
        if (!cam1_ok) std::cerr << "摄像头1（设备" << CAM1_DEVICE << "）：/dev/video" << CAM1_DEVICE << " 无法访问" << std::endl;
        if (!cam2_ok) std::cerr << "摄像头2（设备" << CAM2_DEVICE << "）：/dev/video" << CAM2_DEVICE << " 无法访问" << std::endl;
        std::cerr << "请执行 `v4l2-ctl --list-devices` 确认设备节点！" << std::endl;
        return -1;
    }

    // 等待摄像头稳定（CSI摄像头需要更长时间）
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    cv::Mat frame1, frame2, combined_frame;
    std::cout << "双摄像头启动成功！按ESC键退出..." << std::endl;
    std::cout << "摄像头1：/dev/video" << CAM1_DEVICE << " | 摄像头2：/dev/video" << CAM2_DEVICE << std::endl;







// 摄像头1、2初始化结束================================================================


	tiaosu = 3500;
	speed = -3500;
	std::string picpath = "/home/pi/car_new_new/051.jpg";
	cv::Mat imagege = cv::imread(picpath);


	ArrowDetectorParams params;
  	ArrowDetector detector(params);


// 主循环开始================================================================


	while (xunhuan == 1) {

//		auto loop_start = std::chrono::high_resolution_clock::now();




		if (!capture1.read(frame1)||!capture2.read(frame2)) {
			std::cout << "Can not read frame from video capture!" << std::endl;
			break;
		} 

			if (debug_mode) {
				detectConestart = 1;
				speed = 0;
			}


if (yuyinflag == 2) {

    std::cout << "正在播放语音" << std::endl;
	system("amixer -c 2 set PCM 100% unmute");
    system("amixer -q set PCM 180%");
    system("aplay -D plughw:2,0 /home/pi/2024car/output.wav");


    yuyinflag = 200;   // 立刻禁止再次触发
    BMGet = 0;
    banmaenable = 0;
    A4shibie = 1;
    directionclock = 1;

    std::cout << "语音播放完成" << std::endl;
}

// 引导区操作——————————————————————————————————————————————————————————————————————————————————————————————————————————

            // ========== YOLO AB检测 ==========
           if (yoloABDetectEnable == 1) {
                // 使用 frame2（第二个摄像头）进行检测
				std::cout << "YOLO检测AB" << std::endl;
                auto detections = yoloABDetector.infer(frame2);
                
                // 计算画面垂直中心位置
                int frame_center_x = frame2.cols / 2;
                
                // 用于存储距离中心最近的有效目标
                float min_distance = std::numeric_limits<float>::max();
                std::string best_label = "";
                float best_confidence = 0.0f;
                
                // 遍历所有检测结果，找到距离中心最近的有效目标
                for (const auto& det : detections) {
                    std::cout << "检测到: " << det.label 
                              << " 置信度: " << det.confidence 
                              << " 位置: (" << det.box.x << ", " << det.box.y << ")" 
                              << std::endl;
                    
                    // 只处理置信度大于0.6的检测结果
                    if (det.confidence > 0.6f && (det.label == "A" || det.label == "B")) {
                        // 计算检测框中心的x坐标
                        float box_center_x = det.box.x + det.box.width / 2.0f;
                        
                        // 计算到画面中心的距离
                        float distance = std::abs(box_center_x - frame_center_x);
                        
                        std::cout << "  - 目标中心x: " << box_center_x 
                                  << ", 距离画面中心: " << distance << std::endl;
                        
                        // 如果这个目标更接近中心，则更新最佳目标
                        if (distance < min_distance) {
                            min_distance = distance;
                            best_label = det.label;
                            best_confidence = det.confidence;
                        }
                    }
                }
                
                // 根据距离中心最近的目标设置结果
                if (!best_label.empty()) {
                    if (best_label == "A") {
                        yoloABOver = 1;  // 检测到A
                        left_or_right = 1;
                        std::cout << "==> 最终选择: YOLO检测到 A (置信度: " << best_confidence 
                                  << ", 距中心: " << min_distance << ")" << std::endl;
                    } else if (best_label == "B") {
                        yoloABOver = 1;  // 检测到B
                        left_or_right = 2;
                        std::cout << "==> 最终选择: YOLO检测到 B (置信度: " << best_confidence 
                                  << ", 距中心: " << min_distance << ")" << std::endl;
                    }
					Aflag = 0;
					Bflag = 0;
					ABcount = 0;
					yoloABDetectEnable = 0;  // 检测完成，关闭检测
                } else {
                    std::cout << "==> 未检测到有效的AB目标" << std::endl;
                }
                
				// if(ABcount==3){

				// 	if(Aflag>Bflag){
				// 		yoloABOver = 1;  // 检测到A
				// 		left_or_right=1;
				// 		std::cout << "最终判定 YOLO检测到 A!" << std::endl;
				// 	}
				// 	if(Aflag<Bflag){
				// 		yoloABOver = 1;  // 检测到B
				// 		left_or_right=2;
				// 		std::cout << "最终判定 YOLO检测到 B!" << std::endl;
				// 	}
				// 	Aflag=0;
				// 	Bflag=0;
				// 	ABcount=0;
				// 	yoloABDetectEnable=0;  // 检测完成，关闭检测
				// }
            }

            // ========== YOLO 箭头检测 ==========
            if (yoloArrowDetectEnable == 1) {
				manbastart = 0;
                auto arrowDetections = yoloArrowDetector.infer(frame2);
                
                for (const auto& det : arrowDetections) {
                    std::cout << "箭头检测到: " << det.label 
                              << " 置信度: " << det.confidence << std::endl;
                    
                    if (det.label == "A" && det.confidence > 0.6f) {
						// Lflag++;
						// LRcount++;
//                      yoloArrowResult = 1;  // 检测到左箭头

						yoloArrowResult = 1;  // 检测到左箭头
//						directionclock = 1;
						LRoutput114 = 1;
                        std::cout << "YOLO检测到 左箭头!" << std::endl;
                    } else if (det.label == "B" && det.confidence > 0.6f) {
						// Rflag++;
						// LRcount++;
//                     	yoloArrowResult = 2;  // 检测到右箭头

						yoloArrowResult = 2;  // 检测到右箭头
//						directionclock = 1;
						LRoutput114 = 2;
                        std::cout << "YOLO检测到 右箭头!" << std::endl;
                    }
                }
				Lflag=0;
				Rflag=0;
				LRcount=0;
				yoloArrowDetectEnable=0;  // 检测完成，关闭检测
				yuyinflag = 2; // 2是允许
				std::cout << "开始播放语音" << std::endl;
// 				if(LRcount==3){

// 					if(Lflag>Rflag){
// 						yoloArrowResult = 1;  // 检测到左箭头
// //						directionclock = 1;
// 						LRoutput114 = 1;
// 						std::cout << "最终判定 YOLO检测到 左箭头!" << std::endl;
// 					}
// 					if(Lflag<Rflag){
// 						yoloArrowResult = 2;  // 检测到右箭头
// //						directionclock = 1;
// 						LRoutput114 = 2;
// 						std::cout << "最终判定 YOLO检测到 右箭头!" << std::endl;
// 					}
// 					Lflag=0;
// 					Rflag=0;
// 					LRcount=0;
// 					yoloArrowDetectEnable=0;  // 检测完成，关闭检测
// 					yuyinflag = 2; // 2是允许
// 					std::cout << "开始播放语音" << std::endl;
// 				}
            }



			if (directionclock == 1) {

				if (LRoutput114 == 1) // 箭头为左转
				{
					yindaoquBZ = 1; // 引导区避障标志位置2
					LRoutput = 2;	// 换道左
					std::cout << "换道方向为: 左" << std::endl;
					hdflage = 1;
					directionclock = 0;

				} else if (LRoutput114 == 2) // 箭头为右转
				{
					LRoutput = 1;
					yindaoquBZ = 2; // 引导区避障标志位置1
					std::cout << "换道方向为: 右" << std::endl;
					hdflage = 1;
					directionclock = 0;

				}
			}

			if (yindaoqustart == 1) // 2、根据赛道箭头提示进行避障（调用避障算法）
			{
				if (yindaoquBZ == 1) // 箭头为左转
				{

					std::cout << "箭头左避障右" << std::endl;
					leftrightflag = 1;

				} else if (yindaoquBZ == 2) // 箭头为右转
				{

					std::cout << "箭头右避障左" << std::endl;
					leftrightflag = 2;
				}
				detectConestart = 1;
				yindaoqustart = 0;
			}

			if (detectConestart == 1) { // 引导区开始标志位

				cone_result = detectConeGuideTail(frame1, 88, debug_mode ? true : false);
				
				std::cout << "cone_result: " << cone_result << std::endl;
			}




			if (cone_result == 1) // 引导区开始标志位
			{

				cout << "识别到尾部锥桶" << std::endl;
				if (leftrightflag == 1) // 箭头左避障右
				{
					LRoutput2 = 2;
					std::cout << "锥桶引导右" << std::endl;
				} else if (leftrightflag == 2) // 箭头右避障左
				{
					LRoutput2 = 1;
					std::cout << "锥桶引导左" << std::endl;
				}
			}

			if (SwitchLaneover == 1) {
				LRoutput2 = 0;
				detectConestart = 0;
			}

// 引导区操作完——————————————————————————————————————————————————————————————————————————————————————————————————————————
// 入库操作——————————————————————————————————————————————————————————————————————————————————————————————————————————————
			
	if(rukustart ==1)
	{
			
			std::cout << "进入入库处理函数" << std::endl;
// 		if(CAR_STOP_FLAG == 1){
// //			motorSet(4000);
// //			servor_set(65);
// //			std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // 延时 1000 毫
// //			speed= 4000;
// //			yoloABDetectEnable=1; //开启yoloAB检测
// //			CAR_STOP_FLAG = 0;
// 		}

			if (left_or_right==1 && yoloABOver==1)
			{
				CAR_STOP_FLAG = 0;
				std::cout << "开始入左库" << std::endl;
				motorSet(2000);
				servor_set(75);
				time_sleep(0.9);
				motorSet(-1500);
				servor_set(75);
				time_sleep(0.9);
				motorSet(-1500);
				servor_set(55);
				time_sleep(0.9);
				motorSet(-1500);
				servor_set(65);
				time_sleep(0.9);
				// servor_set(65);
				// time_sleep(0.9);				
				stopflag1=1;
				ABoutput1=1;//----------------------------------这里修改左右入库
			}
			if (left_or_right==2 && yoloABOver==1)
			{
				CAR_STOP_FLAG = 0;
				std::cout << "开始入右库" << std::endl; 
				motorSet(2000);
				servor_set(55);
				time_sleep(0.9);
				motorSet(-1500);
				servor_set(55);
				time_sleep(0.9);
				motorSet(-1500);
				servor_set(75);
				time_sleep(0.9);
				motorSet(-1500);
				servor_set(65);
				time_sleep(0.9);
				// servor_set(65);
				// time_sleep(0.9);		
				stopflag1=1;
				ABoutput1=1;//----------------------------------这里修改左右入库
			}
		

	}
        if(stopflag1==1)
        { 
            xunhuan=2;
            speed=-1000;
            tiaosu=1000;
            clock2=TIMEDELAY;
            break;
        }





// // 避障降速操作——————————————————————————————————————————————————————————————————————————————————————————————————————————————
// 		if(bizhang_get==1 && TIMEDELAY < 25)
// 		{
// //			speed=-1500;
// //			tiaosu=1500;
// 			std::cout << "开始避障降速" << std::endl;
// 			bizhangtime = TIMEDELAY;
// 			bizhang_get = 0;
// 		}

// 		if (TIMEDELAY - bizhangtime > 3 && bizhangover == 0) {
// 			speed = -3500;
// 			tiaosu = 3500;
// 			bizhangover = 1;
// 			bizhangenable = 0;
// 			std::cout << "避障完成，恢复速度 111111111111111111111111111" << std::endl;
// 		}
// // 入库操作完——————————————————————————————————————————————————————————————————————————————————————————————————————————————



			car_error = TUxiang_Init(frame1);
			std::cout << "error: " << car_error << std::endl;

			if (fangxiang == 1) {
				pidcont(car_error);
			}
			banmachuli();
			huandao();
			SwitchLane(LRoutput2); // 根据锥桶引导结果换道
			motorSet(speed);	   // 车速控制1200

			// 3、完成换道后，继续循迹（看到和赛道边界接近的锥桶坐标不予理会）
			// 如果后续发现会把赛道两边的锥桶识别为赛道内部障碍物就考虑加一个简单计时
			// 4、看到赛道中间的锥桶，祈求学长的避障的代码发力


			// // 读取摄像头捕获的图像
			// if (!capture.read(frame)) {
			// 	std::cout << "Can not read frame from video capture!" << std::endl;
			// 	break;
			// } else {
			// 	// std::cout << "I have read the frame from video capture!" << std::endl;

			// 	int car_error = TUxiang_Init(frame);

			// 	std::cout << "car_error 的值为: " << car_error << std::endl;

			// 	pidcont(car_error);
			// 	banmachuli();
			// 	motorSet(speed);

				// auto loop_end = std::chrono::high_resolution_clock::now();
				// auto loop_duration =
				// 	std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start);
				// double loop_time_ms = loop_duration.count() / 1000.0;
				// double frequency = 1000.0 / loop_time_ms; // Hz
				// std::cout << "当前循环耗时: " << loop_duration.count() << " μs (" << loop_time_ms
				// 		  << " ms) | 频率: " << frequency << " Hz" << std::endl;
			// }
			// 等待按键，获取用户输入

//					cv::imshow("Camera 1", frame1);
//					cv::imshow("Camera 2", frame2);

			if (cv::waitKey(1) == 27) {
				break;
			}
	}
	while (xunhuan==2)
    {
        // 读取摄像头捕获的图像
		if (!capture1.read(frame1)||!capture2.read(frame2)) 
        {
            std::cout << "Can not read frame from video capture1!" << std::endl;
            break;
        }
        else
        {

            int car_error = TUxiang_Init(frame1);

            std::cout << "in" << std::endl;
            if(left_or_right=1)//左库
            {errchange1 = -8;}
            if(left_or_right=2)//右库
            {errchange1=10;}
            pidcont(car_error);
            
            if(stopcar==1)
            {motorSet(speed);}
            int clock1=TIMEDELAY; 
            int z;

            clocke=clock1-clock2;
            std::cout << "时差是 ="<< clocke << std::endl;

			if (clocke >= 0) 
			{   
				motorSet(2500);
				stopcar=0;
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			
				cv::imshow("yuantu ",frame2);
				cv::waitKey(10);
				speed=0;
				ABoutput1=0;
				
			}


		}


	}

	capture1.release(); // 释放摄像头
    capture2.release();       // 释放第二个摄像头

	cv::destroyAllWindows(); // 关闭所有窗口

	return 0;
}

int ALLflag = 0;
int TIMEDELAY = 0; // 定时器延时

std::mutex mtx; // 全局互斥锁对象

void TimerInterrupt() // 定时器中断
{

	mtx.lock();
	BANMATIME++;
	if (time_flagggggg == 0) {
		TIMEDELAY++;
	}
	mtx.unlock();
	// 定时器中断处理函数
	std::cout << "定时器触发!!!!!!!!!!!! =  %d" << TIMEDELAY << std::endl;
}
//----------------------------------------------------------

int timedelayIT(void) // 定时器
{
	while (true) {
		TimerInterrupt();
		std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 延时 1000 毫  ooooo

		// 只调避障
		//  if(TIMEDELAY==5)
		//  {
		//          speed=-3000;
		//          tiaosu=3000;
		//          bizhangenable=1;
		//  }

		// if(TIMEDELAY==41)
		// {

		//     tiaosu =1500;
		//     speed=-1500;
		//     errchange1=4;
		//     banmaenable=0;
		//     panduanenable=0;
		//     LRenable=0;
		//     bizhangenable=0;
		// }

		// 只调入库
		//  if(TIMEDELAY==5)
		//  {
		//     errchange1=4;
		//      speed=-1500;
		//      tiaosu = 1500;
		//      panduanenable=1;
		//      //ABenable=1;
		//      yellowenable=1;
		//      std::cout << "速度已经切换到1500" << std::endl;
		//      std::cout << "黄线检测已打开" << std::endl;
		//      std::cout << "AB检测已打开" << std::endl;
		//  }

		// //只调斑马
		// if(TIMEDELAY==5)
		// {
		//         errchange1=5;
		//         speed=-2500;
		//         tiaosu=2500;
		//         banmaenable=1;
		// }

		// 避障和入库
// 		 if(TIMEDELAY==15)
// 		 {
// 		         speed=-2500;
// 		         tiaosu=2500;
// //		         bizhangenable=1;
// 		 }

		//     if(TIMEDELAY==15)
		//     {
		//        errchange1=2;
		//         speed=-1500;
		//         tiaosu = 1500;
		//         panduanenable=1;
		//         //ABenable=1;
		//         yellowenable=1;
		//         std::cout << "速度已经切换到1500" << std::endl;
		//         std::cout << "黄线检测已打开" << std::endl;
		//         std::cout << "AB检测已打开" << std::endl;
		//     }

		// 斑马线避障停车
		//       if(TIMEDELAY==5)//27
		//      {
		//          errchange1=5;
		//          speed=-3000;
		//          tiaosu=3000;//126546853
		//          banmaenable=1;
		//          time_flagggggg=1;
		//          TIMEDELAY++;
		//    }
		//          if(TIMEDELAY==7)//27
		//      {
		//          speed=-3000;
		//          tiaosu=3000;
		//      }
		//          if(TIMEDELAY==8)//27
		//      {
		//          std::cout << "使能图像识别" << std::endl;
		//          std::cout << "使能左右判断" << std::endl;
		//          scan=1;
		//          LRenable=1;
		//          banmaenable=0;
		//      }
		//   if(TIMEDELAY==10)
		//      {
		//          std::cout << "使能避障" << std::endl;
		//          bizhangenable=1;
		//          banmaenable=0;
		//      }
		//  if(TIMEDELAY==22)
		//  {

		//     tiaosu =1500;
		//     speed=-1500;
		//     errchange1=4;
		//     banmaenable=0;
		//     panduanenable=0;
		//     LRenable=0;
		//     bizhangenable=0;
		// }

		// if(TIMEDELAY==22)
		// {

		//     speed=-1500;
		//     tiaosu = 1500;
		//     panduanenable=1;
		//     //ABenable=1;
		//     yellowenable=1;
		//     std::cout << "速度已经切换到1500" << std::endl;
		//     std::cout << "黄线检测已打开" << std::endl;
		//     std::cout << "AB检测已打开" << std::endl;
		// }

		// // 全程
		// if (TIMEDELAY == 5) // 27
		// {
		//   std::cout << "使能斑马线" << std::endl;
		//   std::cout << "使能图像识别" << std::endl;
		//   std::cout << "使能左右判断" << std::endl;
		//   speed = -4000;
		//   tiaosu = 5000;
		// }
		// if (TIMEDELAY == 19) {
		//   motorSet(3000);
		//   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		//   tiaosu = 4000;
		// }
		// if (TIMEDELAY == 20) {
		//   speed = -3500;
		//   tiaosu = 4000;
		// }
		// if (TIMEDELAY == 21) {
		//   motorSet(3000);
		//   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		//   tiaosu = 3500;
		// }
		// if (TIMEDELAY == 22) {
		//   speed = -3000;
		//   tiaosu = 3000;
		// }
		// if (TIMEDELAY == 23) {
		//   motorSet(3000);
		//   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		//   tiaosu = 3000;
		// }
		// if (TIMEDELAY == 24) {
		//   speed = -2500;
		//   tiaosu = 3000;
		// }
		// if (TIMEDELAY == 25) {
		//   errchange1 = 4;
		//   speed = -2500;
		//   tiaosu = 2500; // 126546853
		//   banmaenable = 1;
		//   time_flagggggg = 1;
		//   TIMEDELAY++;
		// }
		// if (TIMEDELAY == 27) // 27
		// {
		//   speed = -3000;
		//   tiaosu = 3000;
		// }

		// if (TIMEDELAY == 28) // 27
		// {
		//   std::cout << "使能图像识别" << std::endl;
		//   std::cout << "使能左右判断" << std::endl;
		//   scan = 1;
		//   LRenable = 1;
		//   banmaenable = 0;
		// }

		// if (TIMEDELAY == 33) {
		//   std::cout << "使能避障" << std::endl;
		//   bizhangenable = 1;
		//   banmaenable = 0;
		// }

		// //     if(TIMEDELAY==40)
		// // {
		// //         motorSet(3000);
		// //         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		// //         tiaosu=1000;
		// // }
		if (TIMEDELAY == 9) { 

		   tiaosu = 2500;
		   speed = -2500;
//		   errchange1 = 0;
		   banmaenable = 1;
		   std::cout << "速度已经切换到2500" << std::endl;
		//   panduanenable = 0;
		//   LRenable = 0;
		   	bizhangenable = 0;
			manbastart = 1;
			bizhangclock = 0;

		  }

		// if (TIMEDELAY == 40) {

		//   speed = -1500;
		//   tiaosu = 1500;
		//   panduanenable = 1;
		//   // ABenable=1;
		//   yellowenable = 1;
		//   std::cout << "速度已经切换到1500" << std::endl;
		//   std::cout << "黄线检测已打开" << std::endl;
		//   std::cout << "AB检测已打开" << std::endl;
		// }
	}
	return 0;
}

int main(void) {
	std::thread thread1(run);

	std::thread thread2(timedelayIT); // 标志识别线程

	thread1.join();
	thread2.join();
	return 0;
}
