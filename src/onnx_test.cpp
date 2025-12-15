#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "onnx_inference.hpp"

#ifdef _WIN32
#define NOMINMAX  // 防止Windows.h定义min/max宏
#include <windows.h>
#endif

std::vector<std::string> listImageFiles(const std::string& dir) {
    namespace fs = std::filesystem;
    static const std::vector<std::string> kExtensions = {
        ".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"};

    std::vector<std::string> files;
    try {
        for (const auto& entry : fs::directory_iterator(dir)) {
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
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 读取目录失败: " << e.what() << std::endl;
        return {};
    }

    std::sort(files.begin(), files.end());
    return files;
}

std::string matShape(const cv::Mat& mat) {
    if (mat.empty()) {
        return "empty";
    }
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < mat.dims; ++i) {
        oss << mat.size[i];
        if (i + 1 < mat.dims) {
            oss << " x ";
        }
    }
    oss << "]";
    return oss.str();
}

void printMinMax(const cv::Mat& mat) {
    if (mat.empty()) {
        std::cout << "    值域: [empty]" << std::endl;
        return;
    }

    // 对于高维 Mat，手动计算 min/max
    if (mat.dims > 2) {
        // 获取连续数据指针
        cv::Mat flat = mat.reshape(1, mat.total());
        double min_v = 0.0, max_v = 0.0;
        cv::minMaxLoc(flat, &min_v, &max_v);
        std::cout << "    值域: [" << min_v << ", " << max_v << "]" << std::endl;
    } else {
        double min_v = 0.0, max_v = 0.0;
        cv::minMaxLoc(mat, &min_v, &max_v);
        std::cout << "    值域: [" << min_v << ", " << max_v << "]" << std::endl;
    }
}

cv::Mat makeHeatmap(const cv::Mat& output, const cv::Size& target_size) {
    cv::Mat map_2d;

    // 常见布局: [N,C,H,W] 或 [H,W]
    if (output.dims == 4 && output.size[1] >= 1) {
        // 取 batch=0, channel=0
        map_2d = cv::Mat(output.size[2],
                         output.size[3],
                         CV_32F,
                         const_cast<float*>(output.ptr<float>(0, 0)));
    } else if (output.dims == 2 && output.rows > 1 && output.cols > 1) {
        map_2d = output.clone();  // H x W 情况
    }

    if (map_2d.empty()) {
        return {};
    }

    cv::Mat norm, heatmap, heatmap_bgr;
    cv::normalize(map_2d, norm, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::applyColorMap(norm, heatmap, cv::COLORMAP_JET);
    cv::resize(heatmap, heatmap_bgr, target_size);
    return heatmap_bgr;
}

// 检测结果结构体: (x, y, w, h, confidence, class_id)
struct Detection {
    float x, y, w, h;  // 归一化坐标
    float confidence;
    int class_id;
};

/**
 * @brief 解析目标检测输出
 * @param output 模型输出 [1, N, 7] 或 [1, N, 6+classes]
 * @param conf_threshold 置信度阈值
 * @return 检测结果列表
 */
std::vector<Detection> parseDetectionOutput(const cv::Mat& output, float conf_threshold = 0.25) {
    std::vector<Detection> detections;

    if (output.dims != 3) {
        return detections;
    }

    int num_detections = output.size[1];
    int num_values = output.size[2];

    std::cout << "[INFO] 检测输出解析:" << std::endl;
    std::cout << "  - 候选框数量: " << num_detections << std::endl;
    std::cout << "  - 每个框的值数量: " << num_values << " (通常: x, y, w, h, conf, class...)" << std::endl;

    // 遍历所有检测结果
    const float* data = output.ptr<float>();
    for (int i = 0; i < num_detections; ++i) {
        const float* det = data + i * num_values;

        // 假设格式: [x, y, w, h, objectness, class_scores...]
        // 或者: [x, y, w, h, conf, class, ...]
        float confidence = (num_values >= 5) ? det[4] : 0.0f;

        if (confidence >= conf_threshold) {
            int class_id = -1;
            float max_class_score = 0.0f;

            // 如果有多个类别分数，找到最大的
            if (num_values > 5) {
                for (int c = 5; c < num_values; ++c) {
                    if (det[c] > max_class_score) {
                        max_class_score = det[c];
                        class_id = c - 5;
                    }
                }
            } else if (num_values >= 6) {
                class_id = static_cast<int>(det[5]);
            }

            // 保存检测结果
            detections.push_back({det[0], det[1], det[2], det[3], confidence, class_id});
        }
    }

    std::cout << "  - 置信度阈值: " << conf_threshold << std::endl;
    std::cout << "  - 有效检测数量: " << detections.size() << std::endl;

    // 显示前 10 个有效检测
    if (!detections.empty()) {
        int show_count = std::min(10, static_cast<int>(detections.size()));
        std::cout << "  - 前 " << show_count << " 个检测结果:" << std::endl;

        for (int i = 0; i < show_count; ++i) {
            const auto& det = detections[i];
            std::cout << "    " << (i + 1) << ". 置信度: " << det.confidence
                      << ", 类别: " << det.class_id
                      << ", bbox: (" << det.x << ", " << det.y
                      << ", " << det.w << ", " << det.h << ")" << std::endl;
        }
    } else {
        std::cout << "  ⚠️  没有检测到任何目标 (置信度 >= " << conf_threshold << ")" << std::endl;
        std::cout << "  建议:" << std::endl;
        std::cout << "    1. 检查预处理是否正确（通道顺序、归一化方式）" << std::endl;
        std::cout << "    2. 尝试降低置信度阈值" << std::endl;
        std::cout << "    3. 使用训练时确认有目标的图像进行测试" << std::endl;
    }

    return detections;
}

/**
 * @brief 在图像上绘制检测框
 * @param image 输入图像（会被修改）
 * @param detections 检测结果列表
 * @param class_names 类别名称列表（可选）
 */
void drawDetections(cv::Mat& image,
                   const std::vector<Detection>& detections,
                   const std::vector<std::string>& class_names = {}) {

    // 为不同类别定义颜色
    std::vector<cv::Scalar> colors = {
        cv::Scalar(0, 255, 0),    // 绿色 - 类别 0
        cv::Scalar(255, 0, 0),    // 蓝色 - 类别 1
        cv::Scalar(0, 0, 255),    // 红色 - 类别 2
        cv::Scalar(255, 255, 0),  // 青色 - 类别 3
        cv::Scalar(255, 0, 255),  // 品红 - 类别 4
        cv::Scalar(0, 255, 255),  // 黄色 - 类别 5
    };

    int img_w = image.cols;
    int img_h = image.rows;

    for (const auto& det : detections) {
        // 转换归一化坐标到像素坐标
        // 假设是中心点格式 (center_x, center_y, width, height)
        int x1 = static_cast<int>((det.x - det.w / 2) * img_w);
        int y1 = static_cast<int>((det.y - det.h / 2) * img_h);
        int x2 = static_cast<int>((det.x + det.w / 2) * img_w);
        int y2 = static_cast<int>((det.y + det.h / 2) * img_h);

        // 限制在图像范围内
        x1 = std::max(0, std::min(x1, img_w - 1));
        y1 = std::max(0, std::min(y1, img_h - 1));
        x2 = std::max(0, std::min(x2, img_w - 1));
        y2 = std::max(0, std::min(y2, img_h - 1));

        // 选择颜色
        cv::Scalar color = colors[det.class_id % colors.size()];

        // 绘制矩形框（加粗边框）
        cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), color, 3);

        // 准备标签文字
        std::string label;
        if (!class_names.empty() && det.class_id < static_cast<int>(class_names.size())) {
            label = class_names[det.class_id];
        } else {
            label = "Class " + std::to_string(det.class_id);
        }
        label += ": " + std::to_string(static_cast<int>(det.confidence * 100)) + "%";

        // 获取文字大小
        int baseline = 0;
        cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);

        // 确保标签在图像内
        int label_y = std::max(y1, label_size.height + 5);

        // 绘制标签背景（半透明效果）
        cv::rectangle(image,
                     cv::Point(x1, label_y - label_size.height - 5),
                     cv::Point(x1 + label_size.width + 5, label_y + baseline),
                     color, cv::FILLED);

        // 绘制标签文字（白色）
        cv::putText(image, label,
                   cv::Point(x1 + 2, label_y - 2),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6,
                   cv::Scalar(255, 255, 255), 2);
    }

    // 在图像顶部显示检测统计
    if (!detections.empty()) {
        std::string stats = "Detections: " + std::to_string(detections.size());
        cv::putText(image, stats,
                   cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 1.0,
                   cv::Scalar(0, 255, 255), 2);
    }
}

void inferAndDisplay(OnnxInference::OnnxModel& model,
                     const cv::Mat& image,
                     const std::string& source_name,
                     const std::string& window_name,
                     bool verbose = true) {
    if (image.empty()) {
        std::cerr << "[ERROR] 输入图像为空: " << source_name << std::endl;
        return;
    }

    try {
        if (verbose) {
            std::cout << "[INFO] 图像: " << source_name
                      << " 大小: " << image.cols << "x" << image.rows << std::endl;
            std::cout << "[INFO] 开始推理..." << std::endl;
        }

        auto start = cv::getTickCount();
        std::vector<cv::Mat> outputs = model.inference(image);
        auto end = cv::getTickCount();
        double time_ms = (end - start) * 1000.0 / cv::getTickFrequency();

        if (verbose) {
            std::cout << "[INFO] 推理完成，耗时: " << time_ms << " ms" << std::endl;
            std::cout << "[INFO] 输出数量: " << outputs.size() << std::endl;
        }

        std::vector<std::pair<int, float>> top5;
        std::vector<Detection> detections;  // 存储检测结果

        if (verbose) {
            for (size_t i = 0; i < outputs.size(); ++i) {
                const auto& output = outputs[i];
                std::cout << "[INFO] 输出 " << i << " - 形状: " << matShape(output)
                          << ", 类型: " << output.type()
                          << ", 总元素数: " << output.total() << std::endl;
                printMinMax(output);

                // 检测输出类型并处理
                if (output.dims == 3 && output.size[2] >= 5) {
                    // 目标检测输出 [1, N, 5+] (YOLO 格式)
                    std::cout << "\n[INFO] 检测到目标检测模型输出格式" << std::endl;
                    detections = parseDetectionOutput(output, 0.25f);
                } else if (output.dims == 2 && (output.rows == 1 || output.cols == 1)) {
                    // 分类模型输出 [1, num_classes]
                    cv::Mat flat = output.reshape(1, output.total());
                    int show_count = std::min(10, static_cast<int>(flat.rows));

                    std::cout << "输出值 (前" << show_count << "个): ";
                    for (int j = 0; j < show_count; ++j) {
                        std::cout << flat.at<float>(j) << " ";
                    }
                    std::cout << std::endl;

                    top5 = model.getTopKClasses(output, 5);
                    std::cout << "[INFO] Top-5 预测:" << std::endl;
                    for (size_t k = 0; k < top5.size(); ++k) {
                        std::cout << "  " << (k + 1) << ". 类别 " << top5[k].first
                                  << ": " << top5[k].second << std::endl;
                    }
                }
            }
        } else if (!outputs.empty()) {
            const auto& output = outputs[0];
            if (output.dims == 3 && output.size[2] >= 5) {
                // 非详细模式下也解析检测结果
                detections = parseDetectionOutput(output, 0.25f);
            } else if (output.dims == 2 && (output.rows == 1 || output.cols == 1)) {
                top5 = model.getTopKClasses(output, 1);
            }
        }

        cv::Mat display_img = image.clone();

        // 绘制检测框（如果有检测结果）
        if (!detections.empty()) {
            // 定义类别名称（根据你的模型修改）
            std::vector<std::string> class_names = {"Left/Right"};  // turnLR 模型
            drawDetections(display_img, detections, class_names);
        }

        // 添加推理时间文字
        cv::putText(display_img,
                    "Inference Time: " + std::to_string(static_cast<int>(time_ms)) + " ms",
                    cv::Point(10, display_img.rows - 20),  // 放在底部
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    cv::Scalar(0, 255, 255),
                    2);

        if (!top5.empty()) {
            std::string label = "Class: " + std::to_string(top5[0].first) +
                                " (" + std::to_string(static_cast<int>(top5[0].second * 100)) + "%)";
            cv::putText(display_img,
                        label,
                        cv::Point(10, 70),
                        cv::FONT_HERSHEY_SIMPLEX,
                        1.0,
                        cv::Scalar(0, 255, 0),
                        2);
        } else {
            cv::putText(display_img,
                        "No output",
                        cv::Point(10, 70),
                        cv::FONT_HERSHEY_SIMPLEX,
                        1.0,
                        cv::Scalar(0, 0, 255),
                        2);
        }

        if (!outputs.empty()) {
            cv::Mat heatmap = makeHeatmap(outputs[0], display_img.size());
            if (!heatmap.empty()) {
                cv::addWeighted(display_img, 0.5, heatmap, 0.5, 0.0, display_img);
            }
        }

        cv::imshow(window_name, display_img);

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 推理失败: " << e.what() << std::endl;
        cv::Mat fallback = image.empty() ? cv::Mat(320, 320, CV_8UC3, cv::Scalar(0, 0, 0))
                                         : image.clone();
        cv::putText(fallback,
                    "Inference failed",
                    cv::Point(10, 40),
                    cv::FONT_HERSHEY_SIMPLEX,
                    1.0,
                    cv::Scalar(0, 0, 255),
                    2);
        cv::imshow(window_name, fallback);
    }
}

/**
 * @brief ONNX模型测试程序
 *
 * 用于测试ONNX模型的加载和推理功能
 * 支持两种测试模式:
 * 1. 使用图像文件进行推理测试
 * 2. 使用摄像头进行实时推理测试
 */

void printUsage(const char* program_name) {
    std::cout << "用法:\n";
    std::cout << "  " << program_name << " <model_path> [image_path] [--size WxH]\n";
    std::cout << "\n";
    std::cout << "参数:\n";
    std::cout << "  model_path   - ONNX模型文件路径 (必需)\n";
    std::cout << "  image_path   - 测试图像路径或目录 (可选，如果不提供则使用摄像头)\n";
    std::cout << "  --size WxH   - 输入图像尺寸，格式: 宽x高 (可选，默认: 28x28)\n";
    std::cout << "\n";
    std::cout << "示例:\n";
    std::cout << "  " << program_name << " model/LR.onnx\n";
    std::cout << "  " << program_name << " model/LR.onnx --size 28x28\n";
    std::cout << "  " << program_name << " model/AB.onnx img/test.jpg\n";
    std::cout << "  " << program_name << " model/AB.onnx img/test.jpg --size 224x224\n";
}

/**
 * @brief 使用图像文件测试模型
 */
void testWithImage(OnnxInference::OnnxModel& model, const std::string& image_path) {
    cv::Mat image = cv::imread(image_path);
    if (image.empty()) {
        std::cerr << "[ERROR] 无法读取图像: " << image_path << std::endl;
        return;
    }

    inferAndDisplay(model, image, image_path, "ONNX Inference Test");
    std::cout << "[INFO] 按任意键退出..." << std::endl;
    cv::waitKey(0);
}

/**
 * @brief 使用图像目录按键浏览测试模型
 */
void testWithImageSequence(OnnxInference::OnnxModel& model,
                           const std::vector<std::string>& image_files) {
    if (image_files.empty()) {
        std::cerr << "[ERROR] 目录中没有可用图片" << std::endl;
        return;
    }

    std::cout << "[INFO] 加载到 " << image_files.size() << " 张图片" << std::endl;
    std::cout << "[INFO] 按空格/Enter/字母n 查看下一张，按 q 或 ESC 退出" << std::endl;

    for (size_t idx = 0; idx < image_files.size(); ++idx) {
        const std::string& path = image_files[idx];
        cv::Mat image = cv::imread(path);
        if (image.empty()) {
            std::cerr << "[WARN] 无法读取图像: " << path << std::endl;
            continue;
        }

        bool advance = false;
        while (!advance) {
            inferAndDisplay(model, image, path, "ONNX Inference Test - Sequence", false);
            int key = cv::waitKey(30);
            if (key == 27 || key == 'q' || key == 'Q') {
                return;
            }
            if (key == ' ' || key == 'n' || key == 'N' || key == '\r') {
                advance = true;
            }
        }
    }
}

/**
 * @brief 使用摄像头测试模型
 */
void testWithCamera(OnnxInference::OnnxModel& model) {
    // 打开摄像头
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "[ERROR] 无法打开摄像头" << std::endl;
        return;
    }

    std::cout << "[INFO] 摄像头已打开，开始实时推理" << std::endl;
    std::cout << "[INFO] 按 'q' 键退出" << std::endl;

    cv::Mat frame;
    bool running = true;

    while (running) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "[WARN] 无法读取帧" << std::endl;
            break;
        }

        try {
            // 执行推理
            auto start = cv::getTickCount();
            std::vector<cv::Mat> outputs = model.inference(frame);
            auto end = cv::getTickCount();

            double time_ms = (end - start) * 1000.0 / cv::getTickFrequency();
            double fps = 1000.0 / time_ms;

            // 解析检测结果（如果是目标检测模型）
            std::vector<Detection> detections;
            if (!outputs.empty()) {
                const auto& output = outputs[0];
                if (output.dims == 3 && output.size[2] >= 5) {
                    // 目标检测模型
                    detections = parseDetectionOutput(output, 0.25f);
                    if (!detections.empty()) {
                        std::vector<std::string> class_names = {"Left/Right"};
                        drawDetections(frame, detections, class_names);
                    }
                } else if (output.dims == 2 && (output.rows == 1 || output.cols == 1)) {
                    // 分类模型，显示Top-1结果
                    auto top1 = model.getTopKClasses(output, 1);
                    if (!top1.empty()) {
                        std::string label = "Class: " + std::to_string(top1[0].first) +
                                          " (" + std::to_string((int)(top1[0].second * 100)) + "%)";
                        cv::putText(frame,
                                   label,
                                   cv::Point(10, 110),
                                   cv::FONT_HERSHEY_SIMPLEX,
                                   1.0,
                                   cv::Scalar(0, 255, 0),
                                   2);
                    }
                }
            }

            // 在图像上显示FPS和推理时间
            cv::putText(frame,
                       "FPS: " + std::to_string((int)fps),
                       cv::Point(10, frame.rows - 60),
                       cv::FONT_HERSHEY_SIMPLEX,
                       0.7,
                       cv::Scalar(0, 255, 255),
                       2);

            cv::putText(frame,
                       "Time: " + std::to_string((int)time_ms) + " ms",
                       cv::Point(10, frame.rows - 30),
                       cv::FONT_HERSHEY_SIMPLEX,
                       0.7,
                       cv::Scalar(0, 255, 255),
                       2);

            cv::imshow("ONNX Real-time Inference", frame);

        } catch (const std::exception& e) {
            std::cerr << "[ERROR] 推理失败: " << e.what() << std::endl;
        }

        // 检查按键
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27) { // 'q' 或 ESC
            running = false;
        }
    }

    cap.release();
    cv::destroyAllWindows();
}

int main(int argc, char* argv[]) {
#ifdef _WIN32
    // 设置Windows控制台为UTF-8编码，解决中文乱码问题
    SetConsoleOutputCP(CP_UTF8);
#endif

    // 检查参数
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string model_path = argv[1];
    std::string image_path;
    int input_width = 320;   // 默认按照模型输入 [1,3,320,320]
    int input_height = 320;

    // 解析命令行参数
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--size" && i + 1 < argc) {
            // 解析尺寸参数，格式: WxH
            std::string size_str = argv[i + 1];
            size_t pos = size_str.find('x');
            if (pos != std::string::npos) {
                try {
                    input_width = std::stoi(size_str.substr(0, pos));
                    input_height = std::stoi(size_str.substr(pos + 1));
                    std::cout << "[INFO] 使用自定义输入尺寸: " << input_width << "x" << input_height << std::endl;
                } catch (...) {
                    std::cerr << "[ERROR] 无效的尺寸格式: " << size_str << std::endl;
                    return 1;
                }
            }
            i++;  // 跳过下一个参数
        } else if (arg.find("--") != 0) {
            // 不是选项参数，当作图像路径
            image_path = arg;
        }
    }

    // 创建ONNX模型对象
    OnnxInference::OnnxModel model(model_path);

    // 加载模型
    if (!model.loadModel()) {
        std::cerr << "[ERROR] 模型加载失败，程序退出" << std::endl;
        return 1;
    }

    // 打印模型信息
    model.printModelInfo();

    // 设置输入大小
    if (input_width != 320 || input_height != 320) {
        std::cout << "[WARN] 当前尺寸与模型导出尺寸(320x320x3)不一致，确认模型是否支持动态尺寸" << std::endl;
    }
    model.setInputSize(input_width, input_height);
    std::cout << "[INFO] 输入图像尺寸设置为: " << input_width << "x" << input_height << " (模型期望 320x320x3)" << std::endl;

    // 启用详细的预处理输出，用于调试
    std::cout << "[INFO] 启用详细预处理输出 (用于调试)" << std::endl;
    model.setVerbosePreprocess(true);

    // 判断测试模式
    if (!image_path.empty()) {
        if (std::filesystem::is_directory(image_path)) {
            auto files = listImageFiles(image_path);
            if (files.empty()) {
                std::cerr << "[ERROR] 目录中没有图片: " << image_path << std::endl;
                return 1;
            }
            std::cout << "[INFO] 使用目录进行测试: " << image_path << std::endl;
            testWithImageSequence(model, files);
        } else {
            std::cout << "[INFO] 使用图像文件进行测试: " << image_path << std::endl;
            testWithImage(model, image_path);
        }
    } else {
        // 使用摄像头测试
        std::cout << "[INFO] 使用摄像头进行实时测试" << std::endl;
        testWithCamera(model);
    }

    std::cout << "[INFO] 测试完成" << std::endl;
    return 0;
}
