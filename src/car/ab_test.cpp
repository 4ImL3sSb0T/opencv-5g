#include "ABDetector.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace {

// Trackbar values for parameters
int blueAreaSlider = 20;   // -> 0.020
int blueFillSlider = 55;   // -> 0.55
int whiteAreaSlider = 5;   // -> 0.005
int morphSlider = 12;      // -> 0.012
int templateThresholdSlider = 60;  // -> 0.60
int detectionMethodSlider = 0;  // 0-3 for different methods
int minHoleAreaSlider = 50;
int maxHoleAreaSlider = 1000;  // x10 in actual value

void initTrackbars() {
  cv::namedWindow("ab_params", cv::WINDOW_NORMAL);
  cv::createTrackbar("blue_area x1000", "ab_params", &blueAreaSlider, 300);
  cv::createTrackbar("blue_fill %", "ab_params", &blueFillSlider, 100);
  cv::createTrackbar("white_area x1000", "ab_params", &whiteAreaSlider, 200);
  cv::createTrackbar("morph_ratio x1000", "ab_params", &morphSlider, 60);
  cv::createTrackbar("template_thresh %", "ab_params", &templateThresholdSlider, 100);
  cv::createTrackbar("detection_method", "ab_params", &detectionMethodSlider, 3);
  cv::createTrackbar("min_hole_area", "ab_params", &minHoleAreaSlider, 500);
  cv::createTrackbar("max_hole_area x10", "ab_params", &maxHoleAreaSlider, 2000);
}

void syncParamsFromTrackbars(ABDetectorParams &params) {
  params.minBlueAreaRatio = std::max(0.001f, blueAreaSlider / 1000.0f);
  params.minBlueFillRatio = std::max(0.10f, blueFillSlider / 100.0f);
  params.minWhiteAreaRatio = std::max(0.0005f, whiteAreaSlider / 1000.0f);
  params.morphKernelRatio = std::max(0.002f, morphSlider / 1000.0f);
  params.templateMatchThreshold = std::max(0.1f, templateThresholdSlider / 100.0f);
  params.minHoleArea = std::max(10, minHoleAreaSlider);
  params.maxHoleArea = std::max(100, maxHoleAreaSlider * 10);
}

ABDetector::DetectionMethod getDetectionMethod() {
  switch (detectionMethodSlider) {
  case 0:
    return ABDetector::DetectionMethod::HoleCount;
  case 1:
    return ABDetector::DetectionMethod::TemplateMatch;
  case 2:
    return ABDetector::DetectionMethod::Projection;
  case 3:
  default:
    return ABDetector::DetectionMethod::Combined;
  }
}

std::vector<std::string> listImageFiles(const std::string &dir) {
  namespace fs = std::filesystem;
  std::vector<std::string> files;
  static const std::vector<std::string> kExtensions = {
      ".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"};

  try {
    for (const auto &entry : fs::directory_iterator(dir)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      std::string ext = entry.path().extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      if (std::find(kExtensions.begin(), kExtensions.end(), ext) !=
          kExtensions.end()) {
        files.push_back(entry.path().string());
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "Failed to read directory: " << e.what() << std::endl;
    return {};
  }

  std::sort(files.begin(), files.end());
  return files;
}

} // namespace

int main(int argc, char **argv) {
  std::string input_source;
  bool debug_mode = true;

  if (argc > 1) {
    input_source = argv[1];
  }
  if (argc > 2) {
    std::string flag = argv[2];
    if (flag == "nodebug") {
      debug_mode = false;
    }
  }

  cv::VideoCapture cap;
  bool use_image_sequence = false;
  std::vector<std::string> image_files;

  if (!input_source.empty()) {
    if (std::filesystem::is_directory(input_source)) {
      image_files = listImageFiles(input_source);
      if (image_files.empty()) {
        std::cerr << "No images found in directory: " << input_source
                  << std::endl;
        return -1;
      }
      use_image_sequence = true;
      std::cout << "Loaded " << image_files.size() << " images from "
                << input_source
                << ". Press space or 'n' for next, 'q'/ESC to quit."
                << std::endl;
    } else {
      cap.open(input_source);
    }
  } else {
    cap.open(0);
  }

  if (!use_image_sequence && !cap.isOpened()) {
    std::cerr << "Failed to open video source." << std::endl;
    return -1;
  }

  ABDetectorParams params;
  if (debug_mode) {
    initTrackbars();
    syncParamsFromTrackbars(params);
  }

  ABDetector detector(params);
  detector.setDebugMode(debug_mode);

  cv::namedWindow("ab_view", cv::WINDOW_NORMAL);

  auto processFrame = [&](const cv::Mat &frame) {
    if (debug_mode) {
      syncParamsFromTrackbars(params);
      detector.setParams(params);
      detector.setDetectionMethod(getDetectionMethod());
    }

    ABDebugInfo debug_info;
    const auto letter =
        detector.detect(frame, debug_mode ? &debug_info : nullptr);

    cv::Mat display = frame.clone();

    // Draw blue region detection
    if (debug_mode && debug_info.blueRect.area() > 0) {
      cv::rectangle(display, debug_info.blueRect, cv::Scalar(0, 255, 0), 2);
    }
    if (debug_mode && debug_info.hasBlueBox) {
      for (int i = 0; i < 4; ++i) {
        cv::line(display, debug_info.blueBox[i],
                 debug_info.blueBox[(i + 1) % 4], cv::Scalar(255, 0, 0), 2);
      }
    }

    // Display detection result
    std::string result_text = "Letter: " + letterToString(letter);
    cv::putText(display, result_text, cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);

    if (debug_mode) {
      // Display method and details
      std::string method_text =
          "Method: " + methodToString(detector.getDetectionMethod());
      cv::putText(display, method_text, cv::Point(20, 80),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

      std::string details;
      if (!debug_info.detectionMethod.empty()) {
        details += debug_info.detectionMethod + " | ";
        details += "Holes:" + std::to_string(debug_info.holeCount);

        // Add hole areas
        if (!debug_info.holeAreas.empty()) {
          details += " [";
          for (size_t i = 0; i < debug_info.holeAreas.size(); ++i) {
            if (i > 0) details += ", ";
            details += std::to_string(static_cast<int>(debug_info.holeAreas[i]));
          }
          details += "]";
        }

        details += " | TmplA:" + cv::format("%.2f", debug_info.templateScoreA) +
                   " TmplB:" + cv::format("%.2f", debug_info.templateScoreB);
      }

      cv::putText(display, details, cv::Point(20, 120),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

      // Show debug windows
      if (!debug_info.blueMask.empty()) {
        cv::imshow("ab_blue_mask", debug_info.blueMask);
      }
      if (!debug_info.whiteMask.empty()) {
        cv::imshow("ab_white_mask", debug_info.whiteMask);

        // Visualize detected holes
        if (!debug_info.holeContours.empty()) {
          cv::Mat hole_vis;
          if (debug_info.whiteMask.channels() == 1) {
            cv::cvtColor(debug_info.whiteMask, hole_vis, cv::COLOR_GRAY2BGR);
          } else {
            hole_vis = debug_info.whiteMask.clone();
          }

          // Draw each hole with different colors and labels
          std::vector<cv::Scalar> colors = {
              cv::Scalar(0, 0, 255),    // Red
              cv::Scalar(0, 255, 0),    // Green
              cv::Scalar(255, 0, 0),    // Blue
              cv::Scalar(255, 255, 0),  // Cyan
              cv::Scalar(255, 0, 255)   // Magenta
          };

          for (size_t i = 0; i < debug_info.holeContours.size(); ++i) {
            cv::Scalar color = colors[i % colors.size()];

            // Draw the hole contour
            cv::drawContours(hole_vis, debug_info.holeContours,
                           static_cast<int>(i), color, 2);

            // Draw hole center and area label
            cv::Moments m = cv::moments(debug_info.holeContours[i]);
            if (m.m00 > 0) {
              int cx = static_cast<int>(m.m10 / m.m00);
              int cy = static_cast<int>(m.m01 / m.m00);
              cv::circle(hole_vis, cv::Point(cx, cy), 3, color, -1);

              std::string label = "H" + std::to_string(i + 1) + ": " +
                                std::to_string(static_cast<int>(debug_info.holeAreas[i]));
              cv::putText(hole_vis, label, cv::Point(cx + 5, cy - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
            }
          }

          cv::imshow("ab_holes", hole_vis);
        }
      }
      if (!debug_info.warpedROI.empty()) {
        cv::imshow("ab_warped_roi", debug_info.warpedROI);
      }

      // Draw projection profile if available
      if (!debug_info.verticalProjection.empty()) {
        int plot_height = 200;
        int plot_width = debug_info.verticalProjection.size();
        cv::Mat plot = cv::Mat::zeros(plot_height, plot_width, CV_8UC3);

        for (size_t i = 0; i < debug_info.verticalProjection.size(); ++i) {
          int h = static_cast<int>(debug_info.verticalProjection[i] *
                                   plot_height);
          cv::line(plot, cv::Point(i, plot_height),
                   cv::Point(i, plot_height - h), cv::Scalar(0, 255, 0), 1);
        }

        cv::imshow("ab_projection", plot);
      }
    }

    cv::imshow("ab_view", display);
  };

  if (use_image_sequence) {
    for (size_t idx = 0; idx < image_files.size(); ++idx) {
      cv::Mat frame = cv::imread(image_files[idx]);
      if (frame.empty()) {
        std::cerr << "Failed to read image: " << image_files[idx] << std::endl;
        continue;
      }

      std::cout << "\n[" << (idx + 1) << "/" << image_files.size()
                << "] Processing: " << image_files[idx] << std::endl;

      bool advance = false;
      while (!advance) {
        processFrame(frame);
        int key = cv::waitKey(30);
        if (key == 27 || key == 'q') { // ESC or q to exit
          return 0;
        }
        if (key == 'n' || key == ' ' || key == '\r') { // next image
          advance = true;
        }
      }
    }
  } else {
    while (true) {
      cv::Mat frame;
      if (!cap.read(frame) || frame.empty()) {
        break;
      }

      processFrame(frame);
      int key = cv::waitKey(1);
      if (key == 27 || key == 'q') { // ESC or q to exit
        break;
      }
    }
  }

  return 0;
}
