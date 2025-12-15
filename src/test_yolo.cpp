#include "yolo_detector.hpp"
#include <iostream>
#include <chrono>

int main() {
    try {
        // 模型路径
        std::string model_path = "../model/AB.onnx";

        // 模型参数（与Python脚本一致）
        int model_w = 320;
        int model_h = 320;
        int num_classes = 2;
        float conf_threshold = 0.5f;
        float nms_threshold = 0.4f;

        // 创建检测器
        std::cout << "Loading YOLO model..." << std::endl;
        YoloInfer::YoloDetector detector(
            model_path,
            model_w,
            model_h,
            num_classes,
            conf_threshold,
            nms_threshold
        );

        // 设置类别标签（与Python脚本一致）
        std::vector<std::string> labels = {"B", "A"};
        detector.setLabels(labels);
        std::cout << "Model loaded successfully!" << std::endl;

        // 读取测试图像
        std::string image_path = "../dataset/AB/images/img_0004_20251210_162928_996.jpg";
        cv::Mat image = cv::imread(image_path);

        if (image.empty()) {
            std::cerr << "Failed to load image: " << image_path << std::endl;
            std::cerr << "Please check the path and file." << std::endl;
            return -1;
        }

        std::cout << "Image loaded: " << image.cols << "x" << image.rows << std::endl;

        // 进行推理并计时
        std::cout << "Running inference..." << std::endl;
        auto t1 = std::chrono::high_resolution_clock::now();

        std::vector<YoloInfer::Detection> detections = detector.infer(image);

        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

        std::cout << "Inference time: " << duration << " ms" << std::endl;
        std::cout << "Detected " << detections.size() << " objects:" << std::endl;

        // 打印检测结果
        for (size_t i = 0; i < detections.size(); ++i) {
            const auto& det = detections[i];
            std::cout << "  [" << i << "] " << det.label
                      << " - confidence: " << det.confidence
                      << " - box: [" << det.box.x << ", " << det.box.y << ", "
                      << det.box.width << ", " << det.box.height << "]" << std::endl;
        }

        // 绘制检测结果
        YoloInfer::YoloDetector::drawDetections(image, detections);

        // 显示结果
        cv::imshow("YOLO Detection Result", image);
        std::cout << "Press any key to exit..." << std::endl;
        cv::waitKey(0);

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
}
