#include "ABDetector.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

ABDetector::ABDetector() = default;

ABDetector::ABDetector(const ABDetectorParams &params) : params_(params) {}

void ABDetector::setParams(const ABDetectorParams &params) {
  params_ = params;
}

ABDetectorParams ABDetector::getParams() const { return params_; }

void ABDetector::setDebugMode(bool enable) { debug_mode_ = enable; }

void ABDetector::setDetectionMethod(DetectionMethod method) {
  detection_method_ = method;
}

ABDetector::DetectionMethod ABDetector::getDetectionMethod() const {
  return detection_method_;
}

int ABDetector::computeKernelSize(const cv::Mat &frame) const {
  int min_dim = std::min(frame.cols, frame.rows);
  int k = static_cast<int>(std::round(params_.morphKernelRatio * min_dim));
  k = std::max(3, k | 1); // force odd size for better morphology
  return k;
}

bool ABDetector::findBlueRegion(const cv::Mat &mask,
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
ABDetector::orderPoints(const cv::Point2f pts[4]) const {
  std::array<cv::Point2f, 4> ordered{};
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

cv::Mat ABDetector::warpBlueRegion(const cv::Mat &frame,
                                   const cv::RotatedRect &rect,
                                   cv::Rect &uprightRect) const {
  cv::Point2f raw_pts[4];
  rect.points(raw_pts);
  auto pts = orderPoints(raw_pts);

  int width = static_cast<int>(std::round(rect.size.width));
  int height = static_cast<int>(std::round(rect.size.height));

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

// Method 1: Detect by counting holes (enclosed regions)
int ABDetector::countHoles(const cv::Mat &whiteMask) const {
  if (whiteMask.empty()) {
    return 0;
  }

  // Find contours with hierarchy to detect holes
  // We use RETR_CCOMP which retrieves all contours and organizes them into a two-level hierarchy
  // External contours (outer boundaries) and hole contours (inner boundaries)
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Mat mask_copy = whiteMask.clone();
  cv::findContours(mask_copy, contours, hierarchy, cv::RETR_CCOMP,
                   cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty() || hierarchy.empty()) {
    return 0;
  }

  // In RETR_CCOMP:
  // hierarchy[i][2] is the first child (hole)
  // hierarchy[i][3] is the parent
  // If hierarchy[i][3] >= 0, this contour is a hole
  int hole_count = 0;
  for (size_t i = 0; i < contours.size(); ++i) {
    // Check if this is a hole (has a parent)
    if (hierarchy[i][3] >= 0) {
      double area = cv::contourArea(contours[i]);
      // Filter by area to avoid noise
      if (area >= params_.minHoleArea && area <= params_.maxHoleArea) {
        hole_count++;
      }
    }
  }

  return hole_count;
}

ABDetector::Letter
ABDetector::detectByHoleCount(const cv::Mat &whiteMask,
                              ABDebugInfo *debugInfo) const {
  if (whiteMask.empty()) {
    return Letter::Unknown;
  }

  // Find contours with hierarchy to detect holes
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Mat mask_copy = whiteMask.clone();
  cv::findContours(mask_copy, contours, hierarchy, cv::RETR_CCOMP,
                   cv::CHAIN_APPROX_SIMPLE);

  // Collect hole information
  std::vector<std::vector<cv::Point>> holes;
  std::vector<double> hole_areas;
  int hole_count = 0;

  if (!contours.empty() && !hierarchy.empty()) {
    for (size_t i = 0; i < contours.size(); ++i) {
      // Check if this is a hole (has a parent)
      if (hierarchy[i][3] >= 0) {
        double area = cv::contourArea(contours[i]);
        // Filter by area to avoid noise
        if (area >= params_.minHoleArea && area <= params_.maxHoleArea) {
          hole_count++;
          holes.push_back(contours[i]);
          hole_areas.push_back(area);
        }
      }
    }
  }

  if (debugInfo) {
    debugInfo->holeCount = hole_count;
    debugInfo->holeContours = holes;
    debugInfo->holeAreas = hole_areas;
    debugInfo->detectionMethod = "HoleCount";
  }

  // A has 1 hole (the triangular space), B has 2 holes (two bumps)
  if (hole_count == 1) {
    return Letter::A;
  } else if (hole_count == 2) {
    return Letter::B;
  }

  return Letter::Unknown;
}

// Method 2: Template matching
cv::Mat ABDetector::createTemplateA(int width, int height) const {
  cv::Mat tmpl = cv::Mat::zeros(height, width, CV_8UC1);

  // Draw a simple "A" shape
  std::vector<cv::Point> pts;
  int cx = width / 2;
  int top_y = height / 5;
  int bottom_y = height * 4 / 5;
  int base_width = width * 3 / 5;

  // Triangle outline
  pts.push_back(cv::Point(cx, top_y));                     // top
  pts.push_back(cv::Point(cx - base_width / 2, bottom_y)); // bottom left
  pts.push_back(cv::Point(cx + base_width / 2, bottom_y)); // bottom right

  // Draw filled triangle
  cv::fillConvexPoly(tmpl, pts, cv::Scalar(255));

  // Draw horizontal bar (middle of A)
  int bar_y = height / 2;
  int bar_thickness = height / 10;
  cv::rectangle(tmpl, cv::Point(cx - base_width / 3, bar_y - bar_thickness / 2),
                cv::Point(cx + base_width / 3, bar_y + bar_thickness / 2),
                cv::Scalar(255), -1);

  // Create hole in middle
  int hole_top = bar_y + bar_thickness;
  int hole_bottom = bottom_y - height / 10;
  std::vector<cv::Point> hole_pts;
  hole_pts.push_back(cv::Point(cx, bar_y));
  hole_pts.push_back(cv::Point(cx - base_width / 6, hole_bottom));
  hole_pts.push_back(cv::Point(cx + base_width / 6, hole_bottom));
  cv::fillConvexPoly(tmpl, hole_pts, cv::Scalar(0));

  return tmpl;
}

cv::Mat ABDetector::createTemplateB(int width, int height) const {
  cv::Mat tmpl = cv::Mat::zeros(height, width, CV_8UC1);

  int bar_width = width / 8;
  int left_x = width / 4;
  int right_x = width * 3 / 4;

  // Draw vertical bar
  cv::rectangle(tmpl, cv::Point(left_x, height / 10),
                cv::Point(left_x + bar_width, height * 9 / 10),
                cv::Scalar(255), -1);

  // Draw two circles for bumps
  int top_center_y = height * 3 / 10;
  int bottom_center_y = height * 7 / 10;
  int radius = height / 5;

  cv::circle(tmpl, cv::Point(left_x + bar_width, top_center_y), radius,
             cv::Scalar(255), -1);
  cv::circle(tmpl, cv::Point(left_x + bar_width, bottom_center_y), radius,
             cv::Scalar(255), -1);

  return tmpl;
}

ABDetector::Letter
ABDetector::detectByTemplateMatch(const cv::Mat &whiteMask,
                                 ABDebugInfo *debugInfo) const {
  if (whiteMask.empty()) {
    return Letter::Unknown;
  }

  // Create templates with same size as input
  cv::Mat templateA = createTemplateA(whiteMask.cols, whiteMask.rows);
  cv::Mat templateB = createTemplateB(whiteMask.cols, whiteMask.rows);

  // Match using normalized cross-correlation
  cv::Mat resultA, resultB;
  cv::matchTemplate(whiteMask, templateA, resultA, cv::TM_CCOEFF_NORMED);
  cv::matchTemplate(whiteMask, templateB, resultB, cv::TM_CCOEFF_NORMED);

  double minValA, maxValA, minValB, maxValB;
  cv::minMaxLoc(resultA, &minValA, &maxValA);
  cv::minMaxLoc(resultB, &minValB, &maxValB);

  if (debugInfo) {
    debugInfo->templateScoreA = maxValA;
    debugInfo->templateScoreB = maxValB;
    debugInfo->detectionMethod = "TemplateMatch";
  }

  // Choose the better match
  if (maxValA > maxValB && maxValA > params_.templateMatchThreshold) {
    return Letter::A;
  } else if (maxValB > maxValA && maxValB > params_.templateMatchThreshold) {
    return Letter::B;
  }

  return Letter::Unknown;
}

// Method 3: Vertical projection analysis
std::vector<double>
ABDetector::computeVerticalProjection(const cv::Mat &whiteMask) const {
  if (whiteMask.empty()) {
    return {};
  }

  std::vector<double> projection(whiteMask.cols, 0.0);

  for (int col = 0; col < whiteMask.cols; ++col) {
    int count = 0;
    for (int row = 0; row < whiteMask.rows; ++row) {
      if (whiteMask.at<uchar>(row, col) > 0) {
        count++;
      }
    }
    projection[col] = static_cast<double>(count) / whiteMask.rows;
  }

  return projection;
}

ABDetector::Letter
ABDetector::detectByProjection(const cv::Mat &whiteMask,
                              ABDebugInfo *debugInfo) const {
  auto projection = computeVerticalProjection(whiteMask);

  if (projection.empty()) {
    return Letter::Unknown;
  }

  if (debugInfo) {
    debugInfo->verticalProjection = projection;
    debugInfo->detectionMethod = "Projection";
  }

  // Analyze projection profile
  // A: Two peaks at sides (legs), valley in middle
  // B: Relatively uniform with bump on right side

  int mid = projection.size() / 2;
  double left_avg = 0.0, right_avg = 0.0, mid_avg = 0.0;

  // Left third
  for (int i = 0; i < mid / 2; ++i) {
    left_avg += projection[i];
  }
  left_avg /= (mid / 2);

  // Middle third
  for (int i = mid / 2; i < mid + mid / 2; ++i) {
    mid_avg += projection[i];
  }
  mid_avg /= mid;

  // Right third
  for (int i = mid + mid / 2; i < static_cast<int>(projection.size()); ++i) {
    right_avg += projection[i];
  }
  right_avg /= (projection.size() - mid - mid / 2);

  // A tends to have higher sides and lower middle
  // B tends to be more uniform
  double side_to_mid_ratio = (left_avg + right_avg) / (2.0 * mid_avg + 1e-6);

  if (side_to_mid_ratio > 1.3) {
    // Strong valley in middle suggests A
    return Letter::A;
  } else if (side_to_mid_ratio < 1.1) {
    // More uniform suggests B
    return Letter::B;
  }

  return Letter::Unknown;
}

// Method 4: Combined approach
ABDetector::Letter
ABDetector::detectByCombined(const cv::Mat &whiteMask,
                            ABDebugInfo *debugInfo) const {
  // Run all methods and vote
  ABDebugInfo temp1, temp2, temp3;

  Letter vote1 = detectByHoleCount(whiteMask, &temp1);
  Letter vote2 = detectByTemplateMatch(whiteMask, &temp2);
  Letter vote3 = detectByProjection(whiteMask, &temp3);

  if (debugInfo) {
    debugInfo->holeCount = temp1.holeCount;
    debugInfo->templateScoreA = temp2.templateScoreA;
    debugInfo->templateScoreB = temp2.templateScoreB;
    debugInfo->verticalProjection = temp3.verticalProjection;
    debugInfo->detectionMethod = "Combined";
  }

  // Simple majority voting
  int votesA = 0, votesB = 0;

  if (vote1 == Letter::A) votesA++;
  else if (vote1 == Letter::B) votesB++;

  if (vote2 == Letter::A) votesA++;
  else if (vote2 == Letter::B) votesB++;

  if (vote3 == Letter::A) votesA++;
  else if (vote3 == Letter::B) votesB++;

  // Require at least 2 votes
  if (votesA >= 2) {
    return Letter::A;
  } else if (votesB >= 2) {
    return Letter::B;
  }

  return Letter::Unknown;
}

// Main detection function
ABDetector::Letter ABDetector::detect(const cv::Mat &frame,
                                     ABDebugInfo *debugInfo) const {
  if (frame.empty()) {
    return Letter::Unknown;
  }

  ABDebugInfo local_debug;
  ABDebugInfo *dbg =
      debugInfo ? debugInfo : (debug_mode_ ? &local_debug : nullptr);

  // Step 1: Convert to HSV
  cv::Mat hsv;
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

  // Step 2: Detect blue background
  cv::Mat blue_mask;
  cv::inRange(hsv, params_.blueLower, params_.blueUpper, blue_mask);

  int k = computeKernelSize(frame);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
  cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_CLOSE, kernel);
  cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_OPEN, kernel);

  // Step 3: Find blue region
  cv::RotatedRect blue_rect;
  if (!findBlueRegion(blue_mask, blue_rect)) {
    if (dbg) {
      dbg->blueMask = blue_mask.clone();
    }
    return Letter::Unknown;
  }

  // Step 4: Warp blue region to get upright ROI
  cv::Rect uprightRect;
  cv::Mat roi_bgr = warpBlueRegion(frame, blue_rect, uprightRect);
  if (roi_bgr.empty()) {
    if (dbg) {
      dbg->blueMask = blue_mask.clone();
    }
    return Letter::Unknown;
  }

  // Step 5: Detect white letters in ROI
  cv::Mat roi_hsv;
  cv::cvtColor(roi_bgr, roi_hsv, cv::COLOR_BGR2HSV);
  cv::Mat white_mask;
  cv::inRange(roi_hsv, params_.whiteLower, params_.whiteUpper, white_mask);

  // Clean up white mask
  int roi_k = std::max(
      3, static_cast<int>(std::round(params_.morphKernelRatio *
                                     std::min(roi_bgr.cols, roi_bgr.rows))));
  roi_k |= 1;
  cv::Mat roi_kernel =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(roi_k, roi_k));
  cv::morphologyEx(white_mask, white_mask, cv::MORPH_OPEN, roi_kernel);
  cv::morphologyEx(white_mask, white_mask, cv::MORPH_CLOSE, roi_kernel);

  // Check if white area is sufficient
  int white_count = cv::countNonZero(white_mask);
  double white_ratio =
      static_cast<double>(white_count) / white_mask.total();
  if (white_ratio < params_.minWhiteAreaRatio) {
    if (dbg) {
      dbg->blueMask = blue_mask.clone();
      dbg->whiteMask = white_mask.clone();
    }
    return Letter::Unknown;
  }

  // Store debug info
  if (dbg) {
    dbg->blueRect = blue_rect.boundingRect();

    cv::Point2f raw_pts[4];
    blue_rect.points(raw_pts);
    auto pts = orderPoints(raw_pts);
    dbg->blueBox = pts;
    dbg->hasBlueBox = true;
    dbg->blueMask = blue_mask.clone();
    dbg->whiteMask = white_mask.clone();
    dbg->warpedROI = roi_bgr.clone();
  }

  // Step 6: Apply selected detection method
  switch (detection_method_) {
  case DetectionMethod::HoleCount:
    return detectByHoleCount(white_mask, dbg);
  case DetectionMethod::TemplateMatch:
    return detectByTemplateMatch(white_mask, dbg);
  case DetectionMethod::Projection:
    return detectByProjection(white_mask, dbg);
  case DetectionMethod::Combined:
    return detectByCombined(white_mask, dbg);
  default:
    return Letter::Unknown;
  }
}
