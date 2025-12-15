#ifndef A4PAPEREXTRACTOR_HPP
#define A4PAPEREXTRACTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>
#include <string>

class A4PaperExtractor {
private:
    bool apply_flip_correction = false;
    std::deque<bool> stability_history;
    bool enableDebug = false;
    
    static constexpr float TARGET_ASPECT = 297.0f / 210.0f;
    static constexpr float CONFIDENCE_THRESHOLD = 0.2f;

public:
    A4PaperExtractor();
    ~A4PaperExtractor();
    
    void setDebugMode(bool enable);
    
    cv::Mat extract(const cv::Mat& frame);
    
private:
    // 图像处理方法
    cv::Mat adaptiveWhiteBalance(const cv::Mat& image);
    cv::Mat detectRedRegionsRobust(const cv::Mat& image);
    cv::Mat completeIncompleteBorder(const cv::Mat& mask, double min_contour_length_ratio = 0.3);
    cv::Mat refineRedMaskAdvanced(const cv::Mat& mask, const cv::Mat& original_image);
    bool detectA4Quadrilateral(const cv::Mat& mask, const cv::Mat& original_image, 
                              std::vector<cv::Point2f>& quad, double aspect_tolerance = 0.4);
    std::vector<cv::Point2f> improvedOrderPoints(const std::vector<cv::Point2f>& pts);
    cv::Mat perspectiveTransform(const cv::Mat& image, const std::vector<cv::Point2f>& quad);
    float calculateConfidence(const cv::Mat& original, const cv::Mat& warped, 
                             const std::vector<cv::Point2f>& quad);
};

#endif // A4PAPEREXTRACTOR_HPP