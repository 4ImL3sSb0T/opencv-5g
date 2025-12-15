#ifndef ONNX_INFERENCE_HPP
#define ONNX_INFERENCE_HPP
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>

namespace OnnxInference {

/**
 * @brief ONNX模型推理类
 *
 * 使用OpenCV DNN模块加载和运行ONNX模型
 */
class OnnxModel {
private:
    cv::dnn::Net net_;
    std::string model_path_;
    bool loaded_;
    std::vector<std::string> output_names_;
    cv::Size input_size_;
    bool verbose_preprocess_{false};

public:
    /**
     * @brief 构造函数
     * @param model_path ONNX模型文件路径
     */
    explicit OnnxModel(const std::string& model_path)
        // 默认按照当前模型的输入形状 [1,3,320,320]
        : model_path_(model_path), loaded_(false), input_size_(320, 320) {
    }

    /**
     * @brief 加载ONNX模型
     * @return 成功返回true，失败返回false
     */
    bool loadModel() {
        try {
            std::cout << "[INFO] 正在加载ONNX模型: " << model_path_ << std::endl;

            // 使用OpenCV DNN模块读取ONNX模型
            net_ = cv::dnn::readNetFromONNX(model_path_);

            if (net_.empty()) {
                std::cerr << "[ERROR] 无法加载ONNX模型: " << model_path_ << std::endl;
                return false;
            }

            // 设置计算后端和目标设备
            // DNN_BACKEND_OPENCV: OpenCV实现
            // DNN_TARGET_CPU: 使用CPU
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

            // 获取输出层名称
            output_names_ = net_.getUnconnectedOutLayersNames();

            loaded_ = true;
            std::cout << "[INFO] ONNX模型加载成功" << std::endl;
            return true;

        } catch (const cv::Exception& e) {
            std::cerr << "[ERROR] 加载ONNX模型时发生OpenCV错误: " << e.what() << std::endl;
            loaded_ = false;
            return false;
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] 加载ONNX模型时发生错误: " << e.what() << std::endl;
            loaded_ = false;
            return false;
        }
    }

    /**
     * @brief 设置输入图像大小
     * @param width 宽度
     * @param height 高度
     */
    void setInputSize(int width, int height) {
        input_size_ = cv::Size(width, height);
    }

    void setInputSize(const cv::Size& size) {
        input_size_ = size;
    }

    cv::Size getInputSize() const {
        return input_size_;
    }

    void setVerbosePreprocess(bool enabled) {
        verbose_preprocess_ = enabled;
    }

    /**
     * @brief 预处理输入图像
     * @param input 输入图像
     * @return 处理后的Blob
     */
    cv::Mat preprocessImage(const cv::Mat& input) {
        cv::Mat blob;

        // 确保输入是 3 通道 BGR，与模型导出时保持一致
        cv::Mat aligned;
        if (input.channels() == 1) {
            cv::cvtColor(input, aligned, cv::COLOR_GRAY2BGR);
        } else if (input.channels() == 4) {
            cv::cvtColor(input, aligned, cv::COLOR_BGRA2BGR);
        } else {
            aligned = input;
        }

        if (verbose_preprocess_) {
            // 计算原始图像统计信息
            cv::Scalar mean, stddev;
            cv::meanStdDev(aligned, mean, stddev);
            double min_val, max_val;
            cv::minMaxLoc(aligned.reshape(1), &min_val, &max_val);

            std::cout << "\n[DEBUG] ========== 预处理详细信息 ==========" << std::endl;
            std::cout << "[DEBUG] 1. 原始图像:" << std::endl;
            std::cout << "[DEBUG]    - 尺寸: " << aligned.cols << "x" << aligned.rows << std::endl;
            std::cout << "[DEBUG]    - 通道数: " << aligned.channels() << std::endl;
            std::cout << "[DEBUG]    - 类型: " << aligned.type() << " (CV_8UC3=16)" << std::endl;
            std::cout << "[DEBUG]    - 值域: [" << min_val << ", " << max_val << "]" << std::endl;
            std::cout << "[DEBUG]    - BGR均值: (" << mean[0] << ", " << mean[1] << ", " << mean[2] << ")" << std::endl;
            std::cout << "[DEBUG]    - BGR标准差: (" << stddev[0] << ", " << stddev[1] << ", " << stddev[2] << ")" << std::endl;
        }

        // 将图像转换为Blob
        // 注意: blobFromImage 的处理顺序是:
        // 1. Resize 到 input_size_
        // 2. 转换为 float 并减去 mean (这里是 0,0,0)
        // 3. 乘以 scalefactor (这里是 1/255)
        // 4. 如果 swapRB=true，交换 R 和 B 通道 (BGR->RGB)
        // 5. 转换为 [N, C, H, W] 格式
        cv::dnn::blobFromImage(aligned, blob,
                               1.0f / 255.0f,      // 归一化到[0,1]
                               input_size_,        // 目标大小 (需与模型输入匹配)
                               cv::Scalar(0, 0, 0),// 均值
                               true,               // 交换RB通道 (BGR->RGB)
                               false,              // 不裁剪
                               CV_32F);            // 浮点类型

        if (verbose_preprocess_) {
            // 计算 blob 统计信息
            // blob 是 4D 的 [N, C, H, W]，需要手动计算 min/max
            const float* blob_data = blob.ptr<float>();
            int total_elements = blob.size[0] * blob.size[1] * blob.size[2] * blob.size[3];

            float blob_min = blob_data[0];
            float blob_max = blob_data[0];
            for (int i = 1; i < total_elements; ++i) {
                blob_min = std::min(blob_min, blob_data[i]);
                blob_max = std::max(blob_max, blob_data[i]);
            }

            // 计算每个通道的均值
            std::vector<float> channel_means(3, 0.0f);
            int channel_size = input_size_.width * input_size_.height;

            for (int c = 0; c < 3; ++c) {
                double sum = 0.0;
                for (int i = 0; i < channel_size; ++i) {
                    sum += blob_data[c * channel_size + i];
                }
                channel_means[c] = sum / channel_size;
            }

            std::cout << "[DEBUG] 2. Blob (blobFromImage 输出):" << std::endl;
            std::cout << "[DEBUG]    - 形状: [" << blob.size[0] << ", " << blob.size[1]
                      << ", " << blob.size[2] << ", " << blob.size[3] << "]" << std::endl;
            std::cout << "[DEBUG]    - 目标尺寸: " << input_size_.width << "x" << input_size_.height << std::endl;
            std::cout << "[DEBUG]    - 缩放因子: 1/255 = " << (1.0f/255.0f) << std::endl;
            std::cout << "[DEBUG]    - 均值减去: (0, 0, 0)" << std::endl;
            std::cout << "[DEBUG]    - 交换RB: true (BGR->RGB)" << std::endl;
            std::cout << "[DEBUG]    - Blob值域: [" << blob_min << ", " << blob_max << "]" << std::endl;
            std::cout << "[DEBUG]    - Blob各通道均值 (RGB): ("
                      << channel_means[0] << ", "
                      << channel_means[1] << ", "
                      << channel_means[2] << ")" << std::endl;
            std::cout << "[DEBUG] ==========================================\n" << std::endl;

            std::cout << "⚠️  如果模型输出异常，请检查:" << std::endl;
            std::cout << "   1. 训练时是否使用了不同的归一化方式 (如 ImageNet标准化)" << std::endl;
            std::cout << "   2. 训练时的通道顺序 (RGB vs BGR)" << std::endl;
            std::cout << "   3. 训练时的输入尺寸是否匹配" << std::endl;
            std::cout << "   4. 可运行: python script/debug_onnx_preprocess.py <image_path>" << std::endl;
            std::cout << std::endl;
        }

        return blob;
    }

    /**
     * @brief 执行推理
     * @param input 输入图像
     * @return 推理结果（可能包含多个输出）
     */
    std::vector<cv::Mat> inference(const cv::Mat& input) {
        if (!loaded_) {
            throw std::runtime_error("模型未加载，请先调用loadModel()");
        }

        if (input.empty()) {
            throw std::invalid_argument("输入图像为空");
        }

        try {
            // 预处理图像
            cv::Mat blob = preprocessImage(input);

            // 设置网络输入
            net_.setInput(blob);

            // 前向传播
            std::vector<cv::Mat> outputs;
            net_.forward(outputs, output_names_);

            return outputs;

        } catch (const cv::Exception& e) {
            std::cerr << "[ERROR] 推理时发生OpenCV错误: " << e.what() << std::endl;
            throw;
        }
    }

    /**
     * @brief 执行推理（简化版本，返回单个输出）
     * @param input 输入图像
     * @return 推理结果
     */
    cv::Mat inferenceSimple(const cv::Mat& input) {
        std::vector<cv::Mat> outputs = inference(input);
        if (outputs.empty()) {
            throw std::runtime_error("推理未返回任何结果");
        }
        return outputs[0];
    }

    /**
     * @brief 从图像路径直接推理
     * @param image_path 图像文件路径
     * @return 推理结果（可能包含多个输出）
     */
    std::vector<cv::Mat> inferenceFromPath(const std::string& image_path) {
        cv::Mat image = cv::imread(image_path);
        if (image.empty()) {
            throw std::runtime_error("无法读取图像: " + image_path);
        }
        return inference(image);
    }

    /**
     * @brief 仅做预处理，返回blob，便于自定义前后处理
     */
    cv::Mat makeBlob(const cv::Mat& input) {
        return preprocessImage(input);
    }

    /**
     * @brief 获取分类结果（用于分类模型）
     * @param output 模型输出
     * @param top_k 返回前k个结果
     * @return 类别索引和置信度的pair向量
     */
    std::vector<std::pair<int, float>> getTopKClasses(const cv::Mat& output, int top_k = 5) {
        std::vector<std::pair<int, float>> results;

        // 确保输出是1D向量
        cv::Mat flat = output.reshape(1, output.total());

        // 创建索引-值对
        std::vector<std::pair<int, float>> index_value_pairs;
        for (int i = 0; i < flat.rows; ++i) {
            index_value_pairs.emplace_back(i, flat.at<float>(i));
        }

        // 按值降序排序
        std::partial_sort(index_value_pairs.begin(),
                         index_value_pairs.begin() + std::min(top_k, (int)index_value_pairs.size()),
                         index_value_pairs.end(),
                         [](const auto& a, const auto& b) { return a.second > b.second; });

        // 返回前k个结果
        for (int i = 0; i < std::min(top_k, (int)index_value_pairs.size()); ++i) {
            results.push_back(index_value_pairs[i]);
        }

        return results;
    }

    /**
     * @brief 检查模型是否已加载
     */
    bool isLoaded() const {
        return loaded_;
    }

    /**
     * @brief 获取输出层名称
     */
    const std::vector<std::string>& getOutputNames() const {
        return output_names_;
    }

    /**
     * @brief 打印模型信息
     */
    void printModelInfo() {
        if (!loaded_) {
            std::cout << "[WARN] 模型未加载" << std::endl;
            return;
        }

        std::cout << "=== ONNX模型信息 ===" << std::endl;
        std::cout << "模型路径: " << model_path_ << std::endl;
        std::cout << "输入大小: " << input_size_.width << "x" << input_size_.height << std::endl;
        std::cout << "输出层数量: " << output_names_.size() << std::endl;

        for (size_t i = 0; i < output_names_.size(); ++i) {
            std::cout << "输出层 " << i << ": " << output_names_[i] << std::endl;
        }
    }
};

} // namespace OnnxInference

#endif // ONNX_INFERENCE_HPP
