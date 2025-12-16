#pragma once

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <vector>
#include <string>
#include <array>

namespace YoloInfer {

// 检测结果结构体
struct Detection {
    cv::Rect box;        // 检测框
    float confidence;    // 置信度
    int class_id;        // 类别ID
    std::string label;   // 类别标签
};

// YOLO检测器类
class YoloDetector {
public:
    /**
     * @brief 构造函数
     * @param model_path ONNX模型路径
     * @param model_w 模型输入宽度
     * @param model_h 模型输入高度
     * @param num_classes 类别数量
     * @param conf_threshold 置信度阈值
     * @param nms_threshold NMS阈值
     */
    YoloDetector(const std::string& model_path,
                 int model_w = 320,
                 int model_h = 320,
                 int num_classes = 2,
                 float conf_threshold = 0.5f,
                 float nms_threshold = 0.4f);

    ~YoloDetector();

    /**
     * @brief 对图像进行推理
     * @param image 输入图像
     * @return 检测结果列表
     */
    std::vector<Detection> infer(const cv::Mat& image);

    /**
     * @brief 设置类别标签
     * @param labels 类别标签字典
     */
    void setLabels(const std::vector<std::string>& labels);

    /**
     * @brief 在图像上绘制检测结果
     * @param image 输入图像
     * @param detections 检测结果
     */
    static void drawDetections(cv::Mat& image, const std::vector<Detection>& detections);

private:
    // 模型参数
    int model_w_;
    int model_h_;
    int num_classes_;
    float conf_threshold_;
    float nms_threshold_;

    // YOLO特定参数
    static constexpr int nl_ = 3;  // 检测层数
    static constexpr int na_ = 3;  // 每层anchor数量
    std::array<float, 3> stride_ = {8.0f, 16.0f, 32.0f};
    std::vector<std::vector<float>> anchors_ = {
        {10, 13, 16, 30, 33, 23},
        {30, 61, 62, 45, 59, 119},
        {116, 90, 156, 198, 373, 326}
    };

    // anchor_grid: [nl, na, 2]
    std::vector<std::vector<std::array<float, 2>>> anchor_grid_;

    // 类别标签
    std::vector<std::string> labels_;

    // ONNX Runtime相关
    Ort::Env env_;
    Ort::Session* session_;
    Ort::SessionOptions session_options_;
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    std::vector<const char*> input_names_ptrs_;
    std::vector<const char*> output_names_ptrs_;
    std::vector<int64_t> input_shape_;

    /**
     * @brief 图像预处理
     * @param image 输入图像
     * @return 预处理后的blob
     */
    std::vector<float> preprocess(const cv::Mat& image);

    /**
     * @brief 创建网格
     * @param w 网格宽度
     * @param h 网格高度
     * @return 网格坐标 [h*w, 2]
     */
    std::vector<std::array<float, 2>> makeGrid(int w, int h);

    /**
     * @brief 计算输出坐标矫正
     * @param outputs 原始输出 [num_predictions, 5+num_classes]
     */
    void calOutputs(std::vector<std::vector<float>>& outputs);

    /**
     * @brief 后处理：NMS和坐标转换
     * @param outputs 模型输出
     * @param img_w 原始图像宽度
     * @param img_h 原始图像高度
     * @return 检测结果
     */
    std::vector<Detection> postprocess(const std::vector<std::vector<float>>& outputs,
                                       int img_w, int img_h);
};

} // namespace YoloInfer
