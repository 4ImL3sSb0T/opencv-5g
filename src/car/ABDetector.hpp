#ifndef ABDETECTOR_HPP
#define ABDETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <array>
#include <string>

struct ABDetectorParams {
  // HSV ranges for the blue background (reuse from ArrowDetector).
  cv::Scalar blueLower{90, 50, 50};
  cv::Scalar blueUpper{140, 255, 255};

  // HSV ranges for the white letters.
  cv::Scalar whiteLower{0, 0, 200};
  cv::Scalar whiteUpper{180, 60, 255};

  // Relative thresholds (normalized to frame size).
  float minBlueAreaRatio = 0.02f;     // min blue area vs full frame
  float minBlueFillRatio = 0.55f;     // contour area vs bounding rect area
  float minWhiteAreaRatio = 0.005f;   // min white area vs blue roi
  float morphKernelRatio = 0.012f;    // kernel size vs min(frame_w, frame_h)

  // Detection method specific parameters
  float templateMatchThreshold = 0.6f;  // for template matching method
  int minHoleArea = 50;                  // minimum hole area for contour method
  int maxHoleArea = 10000;               // maximum hole area for contour method
};

struct ABDebugInfo {
  cv::Rect blueRect;
  std::array<cv::Point2f, 4> blueBox{};
  bool hasBlueBox = false;
  cv::Mat blueMask;
  cv::Mat whiteMask;
  cv::Mat warpedROI;  // warped region for visualization

  // Method-specific debug info
  int holeCount = 0;
  std::vector<std::vector<cv::Point>> holeContours;  // Detected hole contours
  std::vector<double> holeAreas;  // Area of each hole
  double templateScoreA = 0.0;
  double templateScoreB = 0.0;
  std::vector<double> verticalProjection;
  std::string detectionMethod;
};

class ABDetector {
public:
  enum class Letter { Unknown = 0, A = 1, B = 2 };

  enum class DetectionMethod {
    HoleCount = 0,      // Count enclosed regions (holes)
    TemplateMatch = 1,  // Template matching
    Projection = 2,     // Vertical projection analysis
    Combined = 3        // Combined multiple methods
  };

  ABDetector();
  explicit ABDetector(const ABDetectorParams &params);

  void setParams(const ABDetectorParams &params);
  ABDetectorParams getParams() const;

  void setDebugMode(bool enable);
  void setDetectionMethod(DetectionMethod method);
  DetectionMethod getDetectionMethod() const;

  // Main detection function
  Letter detect(const cv::Mat &frame,
                ABDebugInfo *debugInfo = nullptr) const;

private:
  ABDetectorParams params_;
  bool debug_mode_ = false;
  DetectionMethod detection_method_ = DetectionMethod::Combined;

  // Reused from ArrowDetector
  int computeKernelSize(const cv::Mat &frame) const;
  bool findBlueRegion(const cv::Mat &mask, cv::RotatedRect &rect) const;
  std::array<cv::Point2f, 4> orderPoints(const cv::Point2f pts[4]) const;
  cv::Mat warpBlueRegion(const cv::Mat &frame, const cv::RotatedRect &rect,
                         cv::Rect &uprightRect) const;

  // Detection methods
  Letter detectByHoleCount(const cv::Mat &whiteMask,
                          ABDebugInfo *debugInfo) const;
  Letter detectByTemplateMatch(const cv::Mat &whiteMask,
                              ABDebugInfo *debugInfo) const;
  Letter detectByProjection(const cv::Mat &whiteMask,
                           ABDebugInfo *debugInfo) const;
  Letter detectByCombined(const cv::Mat &whiteMask,
                         ABDebugInfo *debugInfo) const;

  // Helper functions
  int countHoles(const cv::Mat &whiteMask) const;
  cv::Mat createTemplateA(int width, int height) const;
  cv::Mat createTemplateB(int width, int height) const;
  std::vector<double> computeVerticalProjection(const cv::Mat &whiteMask) const;
};

// Helper function to convert Letter to string
inline std::string letterToString(ABDetector::Letter letter) {
  switch (letter) {
  case ABDetector::Letter::A:
    return "A";
  case ABDetector::Letter::B:
    return "B";
  default:
    return "Unknown";
  }
}

inline std::string methodToString(ABDetector::DetectionMethod method) {
  switch (method) {
  case ABDetector::DetectionMethod::HoleCount:
    return "HoleCount";
  case ABDetector::DetectionMethod::TemplateMatch:
    return "TemplateMatch";
  case ABDetector::DetectionMethod::Projection:
    return "Projection";
  case ABDetector::DetectionMethod::Combined:
    return "Combined";
  default:
    return "Unknown";
  }
}

#endif // ABDETECTOR_HPP
