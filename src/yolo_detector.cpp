#include "yolo_detector.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>

namespace YoloInfer {

YoloDetector::YoloDetector(const std::string& model_path,
                           int model_w,
                           int model_h,
                           int num_classes,
                           float conf_threshold,
                           float nms_threshold)
    : model_w_(model_w),
      model_h_(model_h),
      num_classes_(num_classes),
      conf_threshold_(conf_threshold),
      nms_threshold_(nms_threshold),
      env_(ORT_LOGGING_LEVEL_WARNING, "YoloDetector"),
      session_(nullptr) {

    // 初始化anchor_grid [nl, na, 2]
    anchor_grid_.resize(nl_);
    for (int i = 0; i < nl_; ++i) {
        anchor_grid_[i].resize(na_);
        for (int j = 0; j < na_; ++j) {
            anchor_grid_[i][j][0] = anchors_[i][j * 2];
            anchor_grid_[i][j][1] = anchors_[i][j * 2 + 1];
        }
    }

    // 配置Session选项
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    // 加载模型
#ifdef _WIN32
    std::wstring w_model_path(model_path.begin(), model_path.end());
    session_ = new Ort::Session(env_, w_model_path.c_str(), session_options_);
#else
    session_ = new Ort::Session(env_, model_path.c_str(), session_options_);
#endif

    // 获取输入输出信息
    Ort::AllocatorWithDefaultOptions allocator;

    // 输入信息
    size_t num_input_nodes = session_->GetInputCount();
    input_names_.resize(num_input_nodes);
    for (size_t i = 0; i < num_input_nodes; i++) {
        auto input_name = session_->GetInputNameAllocated(i, allocator);
        input_names_[i] = std::string(input_name.get());
    }

    // 输出信息
    size_t num_output_nodes = session_->GetOutputCount();
    output_names_.resize(num_output_nodes);
    for (size_t i = 0; i < num_output_nodes; i++) {
        auto output_name = session_->GetOutputNameAllocated(i, allocator);
        output_names_[i] = std::string(output_name.get());
    }

    // 创建指针数组
    input_names_ptrs_.resize(num_input_nodes);
    for (size_t i = 0; i < num_input_nodes; i++) {
        input_names_ptrs_[i] = input_names_[i].c_str();
    }

    output_names_ptrs_.resize(num_output_nodes);
    for (size_t i = 0; i < num_output_nodes; i++) {
        output_names_ptrs_[i] = output_names_[i].c_str();
    }

    // 输入形状
    input_shape_ = {1, 3, model_h_, model_w_};

    // 初始化默认标签
    labels_.resize(num_classes_);
    for (int i = 0; i < num_classes_; ++i) {
        labels_[i] = "class_" + std::to_string(i);
    }
}

YoloDetector::~YoloDetector() {
    if (session_) {
        delete session_;
        session_ = nullptr;
    }
}

void YoloDetector::setLabels(const std::vector<std::string>& labels) {
    labels_ = labels;
}

std::vector<float> YoloDetector::preprocess(const cv::Mat& image) {
    // Resize
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(model_w_, model_h_), 0, 0, cv::INTER_AREA);

    // BGR2RGB
    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

    // 归一化到[0, 1]
    rgb.convertTo(rgb, CV_32F, 1.0 / 255.0);

    // HWC to CHW
    std::vector<float> blob;
    blob.resize(3 * model_h_ * model_w_);

    std::vector<cv::Mat> channels(3);
    cv::split(rgb, channels);

    for (int c = 0; c < 3; ++c) {
        memcpy(blob.data() + c * model_h_ * model_w_,
               channels[c].data,
               model_h_ * model_w_ * sizeof(float));
    }

    return blob;
}

std::vector<std::array<float, 2>> YoloDetector::makeGrid(int w, int h) {
    std::vector<std::array<float, 2>> grid;
    grid.reserve(h * w);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            grid.push_back({static_cast<float>(x), static_cast<float>(y)});
        }
    }

    return grid;
}

void YoloDetector::calOutputs(std::vector<std::vector<float>>& outputs) {
    int row_ind = 0;

    for (int i = 0; i < nl_; ++i) {
        int h = static_cast<int>(model_h_ / stride_[i]);
        int w = static_cast<int>(model_w_ / stride_[i]);
        int length = na_ * h * w;

        // 创建grid
        auto grid = makeGrid(w, h);

        // 处理每个anchor
        for (int a = 0; a < na_; ++a) {
            for (int j = 0; j < h * w; ++j) {
                int idx = row_ind + a * h * w + j;

                // xy坐标解码: (xy * 2 - 0.5 + grid) * stride
                outputs[idx][0] = (outputs[idx][0] * 2.0f - 0.5f + grid[j][0]) * stride_[i];
                outputs[idx][1] = (outputs[idx][1] * 2.0f - 0.5f + grid[j][1]) * stride_[i];

                // wh解码: (wh * 2)^2 * anchor
                outputs[idx][2] = std::pow(outputs[idx][2] * 2.0f, 2) * anchor_grid_[i][a][0];
                outputs[idx][3] = std::pow(outputs[idx][3] * 2.0f, 2) * anchor_grid_[i][a][1];
            }
        }

        row_ind += length;
    }
}

std::vector<Detection> YoloDetector::postprocess(const std::vector<std::vector<float>>& outputs,
                                                  int img_w, int img_h) {
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;

    for (const auto& output : outputs) {
        float confidence = output[4];

        if (confidence < conf_threshold_) {
            continue;
        }

        // 获取类别
        int class_id = 0;
        float max_class_score = output[5];
        for (int i = 1; i < num_classes_; ++i) {
            if (output[5 + i] > max_class_score) {
                max_class_score = output[5 + i];
                class_id = i;
            }
        }

        // 坐标转换到原始图像
        float cx = output[0] / model_w_ * img_w;
        float cy = output[1] / model_h_ * img_h;
        float w = output[2] / model_w_ * img_w;
        float h = output[3] / model_h_ * img_h;

        // 转换为左上角坐标
        int x1 = static_cast<int>(cx - w / 2.0f);
        int y1 = static_cast<int>(cy - h / 2.0f);
        int x2 = static_cast<int>(cx + w / 2.0f);
        int y2 = static_cast<int>(cy + h / 2.0f);

        boxes.push_back(cv::Rect(x1, y1, x2 - x1, y2 - y1));
        confidences.push_back(confidence);
        class_ids.push_back(class_id);
    }

    // NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);

    // 构建结果
    std::vector<Detection> detections;
    for (int idx : indices) {
        Detection det;
        det.box = boxes[idx];
        det.confidence = confidences[idx];
        det.class_id = class_ids[idx];
        det.label = (class_ids[idx] < labels_.size()) ? labels_[class_ids[idx]] :
                    "class_" + std::to_string(class_ids[idx]);
        detections.push_back(det);
    }

    return detections;
}

std::vector<Detection> YoloDetector::infer(const cv::Mat& image) {
    // 预处理
    auto input_tensor_values = preprocess(image);

    // 创建输入tensor
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        input_tensor_values.data(),
        input_tensor_values.size(),
        input_shape_.data(),
        input_shape_.size()
    );

    // 推理
    auto output_tensors = session_->Run(
        Ort::RunOptions{nullptr},
        input_names_ptrs_.data(),
        &input_tensor,
        1,
        output_names_ptrs_.data(),
        1
    );

    // 获取输出
    float* output_data = output_tensors[0].GetTensorMutableData<float>();
    auto output_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();

    // 转换输出格式 [num_predictions, 5 + num_classes]
    int num_predictions = static_cast<int>(output_shape[1]);
    int num_features = static_cast<int>(output_shape[2]);

    std::vector<std::vector<float>> outputs(num_predictions, std::vector<float>(num_features));
    for (int i = 0; i < num_predictions; ++i) {
        for (int j = 0; j < num_features; ++j) {
            outputs[i][j] = output_data[i * num_features + j];
        }
    }

    // 坐标矫正
    calOutputs(outputs);

    // 后处理
    return postprocess(outputs, image.cols, image.rows);
}

void YoloDetector::drawDetections(cv::Mat& image, const std::vector<Detection>& detections) {
    for (const auto& det : detections) {
        // 计算线宽
        int line_thickness = std::max(1, static_cast<int>(std::round(0.002 * (image.rows + image.cols) / 2.0)));

        // 随机颜色（基于class_id）
        cv::Scalar color(
            (det.class_id * 123) % 255,
            (det.class_id * 456) % 255,
            (det.class_id * 789) % 255
        );

        // 绘制矩形
        cv::rectangle(image, det.box, color, line_thickness, cv::LINE_AA);

        // 准备标签
        std::string label = det.label + ":" + cv::format("%.2f", det.confidence);

        // 计算文本大小
        int font_thickness = std::max(line_thickness - 1, 1);
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(
            label,
            cv::FONT_HERSHEY_SIMPLEX,
            line_thickness / 3.0,
            font_thickness,
            &baseline
        );

        // 绘制文本背景
        cv::Point text_org(det.box.x, det.box.y - 3);
        cv::Rect text_rect(
            text_org.x,
            text_org.y - text_size.height,
            text_size.width,
            text_size.height + 3
        );
        cv::rectangle(image, text_rect, color, -1, cv::LINE_AA);

        // 绘制文本
        cv::putText(
            image,
            label,
            cv::Point(det.box.x, det.box.y - 2),
            cv::FONT_HERSHEY_SIMPLEX,
            line_thickness / 3.0,
            cv::Scalar(225, 255, 255),
            font_thickness,
            cv::LINE_AA
        );
    }
}

} // namespace YoloInfer
