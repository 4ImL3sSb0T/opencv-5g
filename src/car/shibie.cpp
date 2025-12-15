#include <iostream>
#include <opencv2/opencv.hpp>
int ABenable = 0;      // AB判断允许（1为允许）
int LRenable = 0;      // LR判断允许（1为允许）
int panduanenable = 1; // 总识别判断允许（1为允许）
int ABvual;            // AB判断的值
int LRvual;            // 左右箭头判断的值
int Aflage = 0;        // 识别为A的次数
int Lflage = 0;        // 识别为L的次数
int Bflage = 0;        // 识别为B的次数
int Rflage = 0;        // 识别为R的次数
int ABround = 0;       // AB轮次
int LRround = 0;       // LR轮次
int ABoutput;          // AB判断最终输出
int LRoutput;          // LR判断最终输出
void panduan() {

  while (panduanenable) {
    //  if(LRenable=1)
    //  { std::cout << "LRenable: " << LRenable << std::endl;  }

    if (ABenable == 1000) // 如果ab允许触发
    {
      // std::cout << "LR处理中" << std::endl;
      if (ABvual == 1) {
        Aflage++;
        std::cout << "AB: " << ABvual << std::endl;
        ABround++;
        ABvual = 0;
      }
      if (ABvual == 2) {
        Bflage++;
        std::cout << "AB: " << ABvual << std::endl;
        ABround++;
        ABvual = 0;
      }

      if (ABround == 3) {

        if (Aflage > Bflage) {
          ABoutput = 1;
          std::cout << "A库" << std::endl;
          ABenable = 0;
          // yellowenable=1;
        }

        if (Aflage < Bflage) {
          ABoutput = 1;
          std::cout << "B库" << std::endl;
          ABenable = 0;
          // yellowenable=1;
        }
      }
    }

    //--------------------------------//
    if (LRenable == 1000) // 如果ab允许触发
    {
      // std::cout << "LR处理中" << std::endl;
      if (LRvual == 1) {
        Lflage++;
        std::cout << "LR: " << LRvual << std::endl;
        LRround++;
        LRvual = 0;
      }
      if (LRvual == 2) {
        Rflage++;
        std::cout << "LR: " << LRvual << std::endl;
        LRround++;
        LRvual = 0;
      }

      if (LRround == 3) {

        if (Lflage > Rflage) {
          LRoutput = 1;
          // std::cout << "右转" << std::endl;
          LRenable = 0;
        }
        if (Lflage < Rflage) {
          LRoutput = 2;
          // std::cout << "左转" << std::endl;
          LRenable = 0;
        }
      }
    }
  }
}
int AB(const cv::Mat &frame1) {
  cv::Mat frame_resized;
  int height = frame1.rows;
  int width = frame1.cols;
  cv::resize(frame1, frame_resized, cv::Size(width / 2, height / 2));

  // hsv 操作
  cv::Mat hsv_image;
  cv::cvtColor(frame_resized, hsv_image, cv::COLOR_BGR2HSV);
  cv::Scalar lower_black(130, 50, 50);
  cv::Scalar upper_black(180, 255, 238);

  // 创建掩膜，提取指定颜色范围内的像素
  cv::Mat mask_BLUE;
  cv::inRange(hsv_image, lower_black, upper_black, mask_BLUE);

  // 查找轮廓
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask_BLUE, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  // 存储四边形的轮廓
  std::vector<std::vector<cv::Point>> quadrilaterals;
  for (const auto &cnt : contours) {
    double epsilon = 0.02 * cv::arcLength(cnt, true);
    std::vector<cv::Point> approx;
    cv::approxPolyDP(cnt, approx, epsilon, true);
    if (approx.size() == 4) {
      double area = cv::contourArea(approx);
      if (area > 400) {
        quadrilaterals.push_back(approx);
        cv::drawContours(frame_resized,
                         std::vector<std::vector<cv::Point>>{approx}, -1,
                         cv::Scalar(255, 0, 0), 2);
      }
    }
  }

  cv::imshow("BZdata", mask_BLUE);

  cv::waitKey(10);
  int AB_check = 0;
  for (const auto &quad : quadrilaterals) {
    // 获取四边形的边界框
    cv::Rect bounding_rect = cv::boundingRect(quad);
    int x = bounding_rect.x;
    int y = bounding_rect.y;
    int w = bounding_rect.width;
    int h = bounding_rect.height;

    // 提取四边形区域
    cv::Mat roi = mask_BLUE(bounding_rect);

    // 在 roi 内查找闭合轮廓
    std::vector<std::vector<cv::Point>> sub_contours;
    cv::findContours(roi, sub_contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    int contour_count = sub_contours.size();
    if (contour_count == 3) {
      AB_check = 1;
    } else if (contour_count == 4) {
      AB_check = 2;
    } else {
      AB_check = 0;
    }
  }

  // OUT
  if (AB_check == 1) {
    // std::cout << "识别结果为 A" << std::endl;
    return 1;
  } else if (AB_check == 2) {
    // std::cout << "识别结果为 B" << std::endl;
    return 2;
  }
  return 3;
}

int arrow(const cv::Mat &input_frame1) {
  // 将帧的尺寸压缩为原来的一半
  cv::Mat frame_resized;
  cv::resize(input_frame1, frame_resized,
             cv::Size(input_frame1.cols / 2, input_frame1.rows / 2));

  // 裁剪去除上二分之一的图像
  int height = frame_resized.rows;
  cv::Mat lower_half = frame_resized(
      cv::Rect(0, height / 4, frame_resized.cols, height - height / 4));

  // hsv 操作，调整颜色范围以更好地检测blue色纸张
  cv::Mat hsv_image;
  cv::cvtColor(lower_half, hsv_image, cv::COLOR_BGR2HSV);
  cv::Scalar lower_red(100, 50, 50);
  cv::Scalar upper_red(130, 255, 255);
  cv::Mat mask_RED;
  cv::inRange(hsv_image, lower_red, upper_red, mask_RED);

  // 形态学操作，进行膨胀和腐蚀以连接可能断开的轮廓和去除小噪声
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::dilate(mask_RED, mask_RED, kernel);
  cv::erode(mask_RED, mask_RED, kernel);

  // 查找轮廓
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask_RED, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  std::string arrow_direction = "";
  for (const auto &cnt : contours) {
    double epsilon = 0.02 * cv::arcLength(cnt, true);
    std::vector<cv::Point> approx;
    cv::approxPolyDP(cnt, approx, epsilon, true);
    if (approx.size() >= 3) {
      double area = cv::contourArea(approx);
      if (area > 400) {
        cv::Rect bounding_rect = cv::boundingRect(approx);
        cv::Mat roi = mask_RED(bounding_rect);

        // 从左到右三等分
        int third_width = bounding_rect.width / 3;
        cv::Mat region1 = roi.colRange(0, third_width);
        cv::Mat region2 = roi.colRange(third_width, 2 * third_width);
        cv::Mat region3 = roi.colRange(2 * third_width, bounding_rect.width);

        // 将下五分之三部分的白色区域转换为红色
        int lower_three_fifths_rows = roi.rows * 2 / 5;
        cv::Mat lower_three_fifths =
            roi.rowRange(lower_three_fifths_rows, roi.rows);
        lower_three_fifths.setTo(0, lower_three_fifths == 255);

        int white_pixels_1 = cv::countNonZero(region1 == 255);
        int white_pixels_2 = cv::countNonZero(region2 == 255);
        int white_pixels_3 = cv::countNonZero(region3 == 255);

        if (white_pixels_1 > white_pixels_2 &&
            white_pixels_1 > white_pixels_3) {
          arrow_direction = "左";
        } else if (white_pixels_3 > white_pixels_1 &&
                   white_pixels_3 > white_pixels_2) {
          arrow_direction = "右";
        }
      }
    }
  }

  if (arrow_direction == "左")
    return 1;
  else if (arrow_direction == "右")
    return 2;
  else
    return 3;
}
