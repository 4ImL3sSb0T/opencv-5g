//==========================头文件区==========================//
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


extern "C" {
#include "serial.h"
}

#include "image_Q.hpp"
#include <thread>


//==========================头文件区==========================//
int speed = -3000;
//==========================定义变量区==========================//

const int pwm_pin = 13;
const float pwm_range = 40000.0;
const float pwm_frequency = 200.0;
const float pwm_duty_cycle_unlock = 10000.0;
const int pwm_pinser = 12;

#define MOTOR_PIN 13
#define SERVO_PIN 12
#define YUNTAI_X_PIN 22 // 云台左右
#define YUNTAI_Y_PIN 23 // 云台上下
#define PWM_BASE 10000  // 电机PWM基准值
#define PWM_RANGE 40000.0
#define PWM_FREQ 200.0
#define SERVO_MID 70 // 舵机中位值

//==========================定义变量区==========================//

//=============================================================//

void motorSet(
    int data) //==========================电机驱动函数==========================//

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

void servor_set(int data) // 舵机值更改  +-18
{
  if (data >= 74 + 12 + 4) {
    data = 74 + 12 + 4;
  }
  if (data <= 74 - 10) {
    data = 74 - 10;
  }
  gpioPWM(12, data);
}

void pwnInit(
    void) //==========================pwm初始化函数==========================//

{
  //    //-----
  if (gpioInitialise() < 0) //==============如果gpioInitialise<0则输出gpio wrong
  {
    std::cout << "GPIO wrong in pwm wrong" << std::endl;
    return;
  } else
    std::cout << "GPIO PWM ok"
              << std::endl; //==============如果正常则输出gpio  wrong
  //--------------
  gpioSetMode(pwm_pinser, PI_OUTPUT);
  gpioSetPWMfrequency(pwm_pinser, 50);
  gpioSetPWMrange(pwm_pinser, 1000);
  gpioPWM(pwm_pinser, 70);

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

void yuntaiInit(
    void) //==========================云台初始化函数==========================//
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
  gpioPWM(22, 77 - 2); // 大左小右(云台左右)
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
int errchange1 = 5;
int tiaosu = 3000;
void pidcont(int Error) // 控制
{
  static int last_Car_Out = 0;
  static int last_error = 0;
  float Car_P = 0; // 0.8
  float Car_D = 0;
  float gkd = 0;
  int Car_Out = 0;
  int mod = 21;
  Error = Error - errchange1; // 减小向左跑
  if (mod == 2) {
    Car_P = 0.2;
    Car_D = 0.3; // 5500
  }

  if (tiaosu == 4000) // 4000速度  初始速度
  {
    Car_P = 0.25;
    Car_D = 0.3;
    errchange1 = 8;
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
    Car_P = 1.0;
    Car_D = 1.2;
  }
  if (tiaosu == 2500) // 2500/2700速度
  {
    Car_P = 0.85;
    Car_D = 1.2;
  }
  if (tiaosu == 3000) // 3000速度
  {
    Car_P = 0.3;
    Car_D = 0.3;
  }
  if (tiaosu == 3700) // 3700速度
  {
    Car_P = 0.59;
    Car_D = 0.9;
  }
  if (tiaosu == 3500) // 3500速度
  {
    Car_P = 0.25;
    Car_D = 0.3;
  }
  if (tiaosu == 1500) // 1500速度
  {
    Car_P = 1.1;
    Car_D = 1.2;
  }
  if (tiaosu == 1000) // 1000速度
  {
    Car_P = 1.2;
    Car_D = 1.2;
  }
  if (tiaosu == 1200) // 1000速度
  {
    Car_P = 1.2;
    Car_D = 1.2;
  }

  Car_Out = -(Car_P * Error + Car_D * (Error - last_error));

  if (Car_Out < -15)
    Car_Out = -15;
  if (Car_Out > 15)
    Car_Out = 15;
  servor_set(Car_Out + 70); // 舵机值  //正左负右
  last_error = Error;
  last_Car_Out = Car_Out;
  //==========================================================
}
int BANMATIME = 0;

int yuyinflag = 0;
int hdflage = 0;
int times;
int stop_banmaxiankaojin = 0;
int y = 0;
void banmachuli(void) {
  if (stopbanma == 1) {
    times = BANMATIME;
    stopbanma = 2;
  }
  if (stopbanma == 2) // 停车保持打角正常
  {
    if (stop_banmaxiankaojin == 0) {
      //  gpioPWM(13,10200);
      //  gpioPWM(13,14200);
      motorSet(4000);
      servor_set(70);
      std::this_thread::sleep_for(
          std::chrono::milliseconds(2500)); // 延时 1000 毫

      times = BANMATIME;
      stop_banmaxiankaojin = 1;
    }
    if (banmaxian_Y < 45) {
      speed = -1200;
      tiaosu = 1200;
      times = BANMATIME;
      std::cout << "times =" << BANMATIME << std::endl;
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
      hdflage = 1;
      yuyinflag = 2; // 2是允许
      std::cout << "现在允许语音了" << std::endl;
    }
  }
}

int stopflage;
int run(void) {
  // 电机舵机初始化================================================================
  gpioTerminate();
  yuntaiInit(); // 云台初始化
  pwnInit();
  // 电机舵机初始化结束=========================================================

  cv::VideoCapture capture;
  capture.open(0);
  if (!capture.isOpened()) {
    std::cout << "Can not open video capture!" << std::endl;
    return -1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 让图像稳定下来
  cv::Mat frame;

  std::string picpath = "/home/pi/car_new_new/051.jpg";
  cv::Mat imagege = cv::imread(picpath);

  while (1) {

    // 读取摄像头捕获的图像
    if (!capture.read(frame)) {
      std::cout << "Can not read frame from video capture!" << std::endl;
      break;
    } else {
      // std::cout << "I have read the frame from video capture!" << std::endl;

      int car_error = TUxiang_Init(frame);

      std::cout << "car_error 的值为: " << car_error - errchange1<< std::endl;

      pidcont(car_error);
      banmachuli();
      motorSet(speed);
    }
    // 等待按键，获取用户输入
    if (cv::waitKey(1) == 27) {
      break;
    }
  }

  capture.release();       // 释放摄像头
  cv::destroyAllWindows(); // 关闭所有窗口

  return 0;
}

int main(void) {
  std::thread thread1(run);
  thread1.join();
  return 0;
}
