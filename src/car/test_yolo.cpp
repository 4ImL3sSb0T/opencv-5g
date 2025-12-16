#include "yolo_detector.hpp"
#include <iostream>
#include <chrono>
#include <filesystem>
#include <algorithm>

namespace fs = std::filesystem;

// 打印使用帮助
void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " --model <model_path> [options]\n\n";
    std::cout << "Options:\n";
    std::cout << "  --model <path>       Model path (required)\n";
    std::cout << "  --camera [id]        Camera mode (default id: 0)\n";
    std::cout << "  --image <path>       Single image mode\n";
    std::cout << "  --dir <path>         Image directory mode (press any key to switch)\n";
    std::cout << "  --conf <value>       Confidence threshold (default: 0.5)\n";
    std::cout << "  --nms <value>        NMS threshold (default: 0.4)\n";
    std::cout << "  --help               Show this help message\n";
    std::cout << "\nExamples:\n";
    std::cout << "  " << program_name << " --model model.onnx --camera\n";
    std::cout << "  " << program_name << " --model model.onnx --camera 1\n";
    std::cout << "  " << program_name << " --model model.onnx --image test.jpg\n";
    std::cout << "  " << program_name << " --model model.onnx --dir ./images\n";
}

// 获取目录下所有图片文件
std::vector<std::string> getImageFiles(const std::string& dir_path) {
    std::vector<std::string> image_files;
    const std::vector<std::string> extensions = {".jpg", ".jpeg", ".png", ".bmp", ".tiff"};

    if (!fs::exists(dir_path) || !fs::is_directory(dir_path)) {
        return image_files;
    }

    for (const auto& entry : fs::directory_iterator(dir_path)) {
        if (entry.is_regular_file()) {
            std::string ext = entry.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

            if (std::find(extensions.begin(), extensions.end(), ext) != extensions.end()) {
                image_files.push_back(entry.path().string());
            }
        }
    }

    std::sort(image_files.begin(), image_files.end());
    return image_files;
}

// 处理单张图片
void processImage(YoloInfer::YoloDetector& detector, const cv::Mat& image,
                  const std::string& title = "YOLO Detection") {
    if (image.empty()) {
        std::cerr << "Empty image!" << std::endl;
        return;
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    auto detections = detector.infer(image);
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

    std::cout << "Inference time: " << duration << " ms - Detected "
              << detections.size() << " objects" << std::endl;

    cv::Mat display = image.clone();
    YoloInfer::YoloDetector::drawDetections(display, detections);

    // 添加FPS信息
    std::string fps_text = "Time: " + std::to_string(duration) + "ms";
    cv::putText(display, fps_text, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

    cv::imshow(title, display);
}

int main(int argc, char* argv[]) {
    try {
        // 解析命令行参数
        std::string model_path;
        std::string mode;
        std::string input_path;
        int camera_id = 0;
        float conf_threshold = 0.5f;
        float nms_threshold = 0.4f;

        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];

            if (arg == "--help" || arg == "-h") {
                printUsage(argv[0]);
                return 0;
            } else if (arg == "--model" && i + 1 < argc) {
                model_path = argv[++i];
            } else if (arg == "--camera") {
                mode = "camera";
                if (i + 1 < argc && argv[i + 1][0] != '-') {
                    camera_id = std::stoi(argv[++i]);
                }
            } else if (arg == "--image" && i + 1 < argc) {
                mode = "image";
                input_path = argv[++i];
            } else if (arg == "--dir" && i + 1 < argc) {
                mode = "directory";
                input_path = argv[++i];
            } else if (arg == "--conf" && i + 1 < argc) {
                conf_threshold = std::stof(argv[++i]);
            } else if (arg == "--nms" && i + 1 < argc) {
                nms_threshold = std::stof(argv[++i]);
            }
        }

        // 检查必需参数
        if (model_path.empty()) {
            std::cerr << "Error: Model path is required!\n\n";
            printUsage(argv[0]);
            return -1;
        }

        if (mode.empty()) {
            std::cerr << "Error: Please specify input mode (--camera, --image, or --dir)\n\n";
            printUsage(argv[0]);
            return -1;
        }

        // 模型参数
        int model_w = 320;
        int model_h = 320;
        int num_classes = 2;

        // 创建检测器
        std::cout << "Loading YOLO model from: " << model_path << std::endl;
        YoloInfer::YoloDetector detector(
            model_path, model_w, model_h, num_classes,
            conf_threshold, nms_threshold
        );

        // 设置类别标签
        std::vector<std::string> labels = {"B", "A"};
        detector.setLabels(labels);
        std::cout << "Model loaded successfully!" << std::endl;
        std::cout << "Conf threshold: " << conf_threshold
                  << ", NMS threshold: " << nms_threshold << std::endl;

        // 根据模式运行
        if (mode == "camera") {
            std::cout << "Starting camera mode (device " << camera_id << ")..." << std::endl;
            std::cout << "Press 'q' or ESC to quit" << std::endl;

            cv::VideoCapture cap(camera_id);
            if (!cap.isOpened()) {
                std::cerr << "Failed to open camera " << camera_id << std::endl;
                return -1;
            }

            cv::Mat frame;
            while (true) {
                cap >> frame;
                if (frame.empty()) {
                    std::cerr << "Failed to capture frame" << std::endl;
                    break;
                }

                processImage(detector, frame, "YOLO Camera Detection");

                int key = cv::waitKey(1);
                if (key == 'q' || key == 27) { // 'q' or ESC
                    break;
                }
            }

            cap.release();

        } else if (mode == "image") {
            std::cout << "Processing single image: " << input_path << std::endl;

            cv::Mat image = cv::imread(input_path);
            if (image.empty()) {
                std::cerr << "Failed to load image: " << input_path << std::endl;
                return -1;
            }

            std::cout << "Image size: " << image.cols << "x" << image.rows << std::endl;
            processImage(detector, image, "YOLO Image Detection");

            std::cout << "Press any key to exit..." << std::endl;
            cv::waitKey(0);

        } else if (mode == "directory") {
            std::cout << "Loading images from directory: " << input_path << std::endl;

            auto image_files = getImageFiles(input_path);
            if (image_files.empty()) {
                std::cerr << "No image files found in: " << input_path << std::endl;
                return -1;
            }

            std::cout << "Found " << image_files.size() << " images" << std::endl;
            std::cout << "Press any key to next image, 'q' to quit" << std::endl;

            size_t current_idx = 0;
            while (current_idx < image_files.size()) {
                const auto& img_path = image_files[current_idx];
                std::cout << "\n[" << (current_idx + 1) << "/" << image_files.size()
                          << "] " << fs::path(img_path).filename().string() << std::endl;

                cv::Mat image = cv::imread(img_path);
                if (image.empty()) {
                    std::cerr << "Failed to load: " << img_path << std::endl;
                    current_idx++;
                    continue;
                }

                // 在图片上添加文件名信息
                std::string info = std::to_string(current_idx + 1) + "/" +
                                   std::to_string(image_files.size()) + ": " +
                                   fs::path(img_path).filename().string();

                processImage(detector, image, "YOLO Directory Detection");

                // 在窗口上添加导航信息
                cv::Mat display;
                cv::namedWindow("YOLO Directory Detection");
                cv::setWindowTitle("YOLO Directory Detection",
                                   ("YOLO - " + info).c_str());

                int key = cv::waitKey(0);
                if (key == 'q' || key == 27) { // 'q' or ESC
                    break;
                } else if (key == 'p' || key == 2424832) { // 'p' or Left arrow
                    if (current_idx > 0) {
                        current_idx--;
                    }
                } else {
                    current_idx++;
                }
            }

            std::cout << "Finished viewing images." << std::endl;
        }

        cv::destroyAllWindows();
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
}
