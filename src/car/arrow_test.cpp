#include "ArrowDetector.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace {

int blueAreaSlider = 20;   // -> 0.020
int blueFillSlider = 55;   // -> 0.55
int whiteAreaSlider = 2;   // -> 0.002
int morphSlider = 12;      // -> 0.012
int contrastSlider = 150;  // -> +1.150

void initTrackbars() {
  cv::namedWindow("arrow_params", cv::WINDOW_NORMAL);
  cv::createTrackbar("blue_area x1000", "arrow_params", &blueAreaSlider, 300);
  cv::createTrackbar("blue_fill %", "arrow_params", &blueFillSlider, 100);
  cv::createTrackbar("white_area x1000", "arrow_params", &whiteAreaSlider, 200);
  cv::createTrackbar("morph_ratio x1000", "arrow_params", &morphSlider, 60);
  cv::createTrackbar("contrast +1000", "arrow_params", &contrastSlider, 2000);
}

void syncParamsFromTrackbars(ArrowDetectorParams &params) {
  params.minBlueAreaRatio = std::max(0.001f, blueAreaSlider / 1000.0f);
  params.minBlueFillRatio = std::max(0.10f, blueFillSlider / 100.0f);
  params.minWhiteAreaRatio = std::max(0.0005f, whiteAreaSlider / 1000.0f);
  params.morphKernelRatio = std::max(0.002f, morphSlider / 1000.0f);
  params.directionContrast = std::max(1.0f, 1.0f + contrastSlider / 1000.0f);
}

std::string directionToString(ArrowDetector::Direction d) {
  switch (d) {
  case ArrowDetector::Direction::Left:
    return "Left";
  case ArrowDetector::Direction::Right:
    return "Right";
  default:
    return "Unknown";
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
      if (std::find(kExtensions.begin(), kExtensions.end(), ext) != kExtensions.end()) {
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
        std::cerr << "No images found in directory: " << input_source << std::endl;
        return -1;
      }
      use_image_sequence = true;
      std::cout << "Loaded " << image_files.size() << " images from " << input_source
                << ". Press space or 'n' for next, 'q'/ESC to quit." << std::endl;
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

  ArrowDetectorParams params;
  if (debug_mode) {
    initTrackbars();
    syncParamsFromTrackbars(params);
  }

  ArrowDetector detector(params);
  detector.setDebugMode(debug_mode);

  cv::namedWindow("arrow_view", cv::WINDOW_NORMAL);

  auto processFrame = [&](const cv::Mat &frame) {
    if (debug_mode) {
      syncParamsFromTrackbars(params);
      detector.setParams(params);
    }

    ArrowDebugInfo debug_info;
    const auto dir =
        detector.detect(frame, debug_mode ? &debug_info : nullptr);

    cv::Mat display = frame.clone();
    if (debug_mode && debug_info.blueRect.area() > 0) {
      cv::rectangle(display, debug_info.blueRect, cv::Scalar(0, 255, 0), 2);
    }
    if (debug_mode && debug_info.hasBlueBox) {
      for (int i = 0; i < 4; ++i) {
        cv::line(display, debug_info.blueBox[i],
                 debug_info.blueBox[(i + 1) % 4], cv::Scalar(255, 0, 0), 2);
      }
    }
    if (debug_mode && debug_info.hasMidLine) {
      cv::line(display, debug_info.midTop, debug_info.midBottom,
               cv::Scalar(0, 255, 255), 2);
    }

    std::string text = "Direction: " + directionToString(dir);
    cv::putText(display, text, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX,
                1.0, cv::Scalar(0, 0, 255), 2);

    if (debug_mode) {
      std::string counts =
          "L:" + std::to_string(debug_info.leftWhiteCount) +
          " R:" + std::to_string(debug_info.rightWhiteCount) +
          " white_ratio:" + cv::format("%.4f", debug_info.whiteRatio);
      cv::putText(display, counts, cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX,
                  0.8, cv::Scalar(255, 255, 0), 2);

      if (!debug_info.blueMask.empty()) {
        cv::imshow("arrow_blue_mask", debug_info.blueMask);
      }
      if (!debug_info.whiteMask.empty()) {
        cv::imshow("arrow_white_mask", debug_info.whiteMask);
      }
    }

    cv::imshow("arrow_view", display);
  };

  if (use_image_sequence) {
    for (size_t idx = 0; idx < image_files.size(); ++idx) {
      cv::Mat frame = cv::imread(image_files[idx]);
      if (frame.empty()) {
        std::cerr << "Failed to read image: " << image_files[idx] << std::endl;
        continue;
      }
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
