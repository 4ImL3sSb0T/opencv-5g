#ifndef ARROWDETECTOR_HPP
#define ARROWDETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <array>

struct ArrowDetectorParams {
  // HSV ranges for the blue background.
  cv::Scalar blueLower{90, 50, 50};
  cv::Scalar blueUpper{140, 255, 255};

  // HSV ranges for the white arrow.
  cv::Scalar whiteLower{0, 0, 200};
  cv::Scalar whiteUpper{180, 60, 255};

  // Relative thresholds (normalized to frame size).
  float minBlueAreaRatio = 0.02f;     // min blue area vs full frame
  float minBlueFillRatio = 0.55f;     // contour area vs bounding rect area
  float minWhiteAreaRatio = 0.002f;   // min white area vs blue roi
  float morphKernelRatio = 0.012f;    // kernel size vs min(frame_w, frame_h)
  float directionContrast = 1.15f;    // min ratio between left/right white counts
};

struct ArrowDebugInfo {
  cv::Rect blueRect;
  std::array<cv::Point2f, 4> blueBox{};
  bool hasBlueBox = false;
  cv::Point2f midTop;
  cv::Point2f midBottom;
  bool hasMidLine = false;
  int leftWhiteCount = 0;
  int rightWhiteCount = 0;
  double whiteRatio = 0.0;
  cv::Mat blueMask;
  cv::Mat whiteMask;
};

class ArrowDetector {
public:
  enum class Direction { Unknown = 0, Left = 1, Right = 2 };

  ArrowDetector();
  explicit ArrowDetector(const ArrowDetectorParams &params);

  void setParams(const ArrowDetectorParams &params);
  ArrowDetectorParams getParams() const;

  void setDebugMode(bool enable);

  Direction detect(const cv::Mat &frame,
                   ArrowDebugInfo *debugInfo = nullptr) const;

private:
  ArrowDetectorParams params_;
  bool debug_mode_ = false;

  int computeKernelSize(const cv::Mat &frame) const;
  bool findBlueRegion(const cv::Mat &mask, cv::RotatedRect &rect) const;
  std::array<cv::Point2f, 4> orderPoints(const cv::Point2f pts[4]) const;
  cv::Mat warpBlueRegion(const cv::Mat &frame, const cv::RotatedRect &rect,
                         cv::Rect &uprightRect) const;
  Direction evaluateDirection(const cv::Mat &whiteMask,
                              ArrowDebugInfo *debugInfo) const;
};

#endif // ARROWDETECTOR_HPP
