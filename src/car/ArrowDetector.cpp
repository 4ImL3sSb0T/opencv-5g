#include "ArrowDetector.hpp"

#include <algorithm>
#include <cmath>

ArrowDetector::ArrowDetector() = default;

ArrowDetector::ArrowDetector(const ArrowDetectorParams &params)
    : params_(params) {}

void ArrowDetector::setParams(const ArrowDetectorParams &params) {
  params_ = params;
}

ArrowDetectorParams ArrowDetector::getParams() const { return params_; }

void ArrowDetector::setDebugMode(bool enable) { debug_mode_ = enable; }

int ArrowDetector::computeKernelSize(const cv::Mat &frame) const {
  int min_dim = std::min(frame.cols, frame.rows);
  int k = static_cast<int>(std::round(params_.morphKernelRatio * min_dim));
  k = std::max(3, k | 1); // force odd size for better morphology
  return k;
}

bool ArrowDetector::findBlueRegion(const cv::Mat &mask,
                                   cv::RotatedRect &rect) const {
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty()) {
    return false;
  }

  double frame_area = static_cast<double>(mask.cols * mask.rows);
  double min_area = params_.minBlueAreaRatio * frame_area;

  double best_area = 0.0;
  cv::RotatedRect best_rect;

  for (const auto &c : contours) {
    double area = cv::contourArea(c);
    if (area < min_area) {
      continue;
    }

    cv::RotatedRect r = cv::minAreaRect(c);
    double rect_area =
        static_cast<double>(std::max(1.0f, r.size.width) *
                            std::max(1.0f, r.size.height));
    double fill_ratio = rect_area > 0.0 ? area / rect_area : 0.0;
    if (fill_ratio < params_.minBlueFillRatio) {
      continue;
    }

    if (area > best_area) {
      best_area = area;
      best_rect = r;
    }
  }

  if (best_area <= 0.0) {
    return false;
  }

  rect = best_rect;
  return true;
}

std::array<cv::Point2f, 4>
ArrowDetector::orderPoints(const cv::Point2f pts[4]) const {
  std::array<cv::Point2f, 4> ordered{};
  // sum = x+y, diff = y-x
  float sums[4], diffs[4];
  for (int i = 0; i < 4; ++i) {
    sums[i] = pts[i].x + pts[i].y;
    diffs[i] = pts[i].y - pts[i].x;
  }

  int tl = std::min_element(sums, sums + 4) - sums;
  int br = std::max_element(sums, sums + 4) - sums;
  int tr = std::min_element(diffs, diffs + 4) - diffs;
  int bl = std::max_element(diffs, diffs + 4) - diffs;

  ordered[0] = pts[tl];
  ordered[1] = pts[tr];
  ordered[2] = pts[br];
  ordered[3] = pts[bl];
  return ordered;
}

cv::Mat ArrowDetector::warpBlueRegion(const cv::Mat &frame,
                                      const cv::RotatedRect &rect,
                                      cv::Rect &uprightRect) const {
  cv::Point2f raw_pts[4];
  rect.points(raw_pts);
  auto pts = orderPoints(raw_pts);

  int width = static_cast<int>(std::round(rect.size.width));
  int height = static_cast<int>(std::round(rect.size.height));

  // ensure positive sizes
  width = std::max(width, 1);
  height = std::max(height, 1);

  std::vector<cv::Point2f> src_pts(pts.begin(), pts.end());
  std::vector<cv::Point2f> dst_pts = {
      cv::Point2f(0.0f, 0.0f),
      cv::Point2f(static_cast<float>(width - 1), 0.0f),
      cv::Point2f(static_cast<float>(width - 1),
                  static_cast<float>(height - 1)),
      cv::Point2f(0.0f, static_cast<float>(height - 1))};

  cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
  cv::Mat warped;
  cv::warpPerspective(frame, warped, M, cv::Size(width, height));

  uprightRect = cv::Rect(0, 0, width, height);
  return warped;
}

ArrowDetector::Direction
ArrowDetector::evaluateDirection(const cv::Mat &whiteMask,
                                 ArrowDebugInfo *debugInfo) const {
  if (whiteMask.empty()) {
    return Direction::Unknown;
  }

  int mid = whiteMask.cols / 2;
  cv::Mat left = whiteMask(cv::Rect(0, 0, mid, whiteMask.rows));
  cv::Mat right =
      whiteMask(cv::Rect(mid, 0, whiteMask.cols - mid, whiteMask.rows));

  int left_count = cv::countNonZero(left);
  int right_count = cv::countNonZero(right);
  double total_white = static_cast<double>(left_count + right_count);
  double roi_area = static_cast<double>(whiteMask.total());
  double white_ratio = roi_area > 0.0 ? total_white / roi_area : 0.0;

  if (debugInfo) {
    debugInfo->leftWhiteCount = left_count;
    debugInfo->rightWhiteCount = right_count;
    debugInfo->whiteRatio = white_ratio;
  }

  if (white_ratio < params_.minWhiteAreaRatio) {
    return Direction::Unknown;
  }

  if (left_count > right_count * params_.directionContrast) {
    return Direction::Left;
  }
  if (right_count > left_count * params_.directionContrast) {
    return Direction::Right;
  }

  return Direction::Unknown;
}

ArrowDetector::Direction
ArrowDetector::detect(const cv::Mat &frame, ArrowDebugInfo *debugInfo) const {
  if (frame.empty()) {
    return Direction::Unknown;
  }

  ArrowDebugInfo local_debug;
  ArrowDebugInfo *dbg = debugInfo ? debugInfo : (debug_mode_ ? &local_debug : nullptr);

  // Crop top 1/3 to avoid sky interference
  int crop_y = frame.rows / 3;
  int crop_height = frame.rows - crop_y;
  cv::Mat cropped_frame = frame(cv::Rect(0, crop_y, frame.cols, crop_height));

  cv::Mat hsv;
  cv::cvtColor(cropped_frame, hsv, cv::COLOR_BGR2HSV);

  cv::Mat blue_mask;
  cv::inRange(hsv, params_.blueLower, params_.blueUpper, blue_mask);

  int k = computeKernelSize(cropped_frame);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
  cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_CLOSE, kernel);
  cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_OPEN, kernel);

  cv::RotatedRect blue_rect;
  if (!findBlueRegion(blue_mask, blue_rect)) {
    if (dbg) {
      dbg->blueMask = blue_mask.clone();
    }
    return Direction::Unknown;
  }

  cv::Rect uprightRect;
  cv::Mat roi_bgr = warpBlueRegion(cropped_frame, blue_rect, uprightRect);
  if (roi_bgr.empty()) {
    if (dbg) {
      dbg->blueMask = blue_mask.clone();
    }
    return Direction::Unknown;
  }

  cv::Mat roi_hsv;
  cv::cvtColor(roi_bgr, roi_hsv, cv::COLOR_BGR2HSV);
  cv::Mat white_mask;
  cv::inRange(roi_hsv, params_.whiteLower, params_.whiteUpper, white_mask);

  // Light clean up using a smaller kernel based on ROI size.
  int roi_k = std::max(3, static_cast<int>(std::round(
                             params_.morphKernelRatio *
                             std::min(roi_bgr.cols, roi_bgr.rows))));
  roi_k |= 1;
  cv::Mat roi_kernel =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(roi_k, roi_k));
  cv::morphologyEx(white_mask, white_mask, cv::MORPH_OPEN, roi_kernel);

  // Focus on upper half to avoid lower-part interference.
  cv::Mat white_mask_upper =
      white_mask.rowRange(0, std::max(1, white_mask.rows / 2));

  if (dbg) {
    // Adjust coordinates back to original frame
    cv::Rect adjusted_rect = blue_rect.boundingRect();
    adjusted_rect.y += crop_y;
    dbg->blueRect = adjusted_rect;

    cv::Point2f raw_pts[4];
    blue_rect.points(raw_pts);
    auto pts = orderPoints(raw_pts);

    // Adjust all corner points
    for (int i = 0; i < 4; ++i) {
      pts[i].y += crop_y;
    }

    dbg->blueBox = pts;
    dbg->hasBlueBox = true;
    dbg->midTop = (pts[0] + pts[1]) * 0.5f;
    dbg->midBottom = (pts[3] + pts[2]) * 0.5f;
    dbg->hasMidLine = true;
    dbg->blueMask = blue_mask.clone();
    dbg->whiteMask = white_mask_upper.clone();
  }

  return evaluateDirection(white_mask_upper, dbg);
}
