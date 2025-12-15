# ONNX 推理使用说明

本项目通过 OpenCV DNN 封装 ONNX 推理。核心封装：`src/onnx_inference.hpp`；命令行示例：`src/onnx_test.cpp`。

## 编译与运行

```bash
cmake -S . -B build
cmake --build build

# 默认输入尺寸 320x320（与当前模型一致）
./onnx_test model/LR.onnx img/test.jpg --size 320x320
```

参数说明：
- `model_path`：ONNX 模型路径。
- `image_path`：可选，测试图片路径；不传则使用摄像头。
- `--size WxH`：可选输入尺寸，默认 320x320。除非模型导出支持动态尺寸，否则请保持 320x320。

## OnnxModel API（`src/onnx_inference.hpp`）

- `OnnxModel model(path)`：创建模型对象，默认输入 320x320。
- `bool loadModel()`：加载 ONNX、设置后端/设备、缓存输出层名。
- `void setInputSize(int w, int h)` / `setInputSize(cv::Size)`：修改输入尺寸（仅在模型支持动态尺寸时使用）。
- `cv::Size getInputSize()`：获取当前输入尺寸。
- `void setVerbosePreprocess(bool)`：开启预处理日志。
- `cv::Mat preprocessImage(const cv::Mat&)` / `makeBlob(...)`：图像转 blob；自动将 1/4 通道转为 3 通道 BGR，并做归一化和缩放。
- `std::vector<cv::Mat> inference(const cv::Mat&)`：执行前向。
- `cv::Mat inferenceSimple(const cv::Mat&)`：返回首个输出（单输出模型常用）。
- `std::vector<cv::Mat> inferenceFromPath(const std::string&)`：直接读图并推理。
- `std::vector<std::pair<int,float>> getTopKClasses(const cv::Mat&, int k)`：分类 Top-K 辅助。
- `printModelInfo()`：打印模型路径、输入尺寸、输出层名。

## 代码示例

```cpp
#include "onnx_inference.hpp"
using namespace OnnxInference;

int main() {
    OnnxModel model("model/LR.onnx");
    model.setVerbosePreprocess(true);     // 可选：打印预处理信息
    model.setInputSize(320, 320);         // 与导出模型保持一致

    if (!model.loadModel()) return -1;
    model.printModelInfo();

    cv::Mat image = cv::imread("img/test.jpg");
    auto outputs = model.inference(image);

    if (!outputs.empty()) {
        auto top5 = model.getTopKClasses(outputs[0], 5);
        for (auto& p : top5) {
            std::cout << "class " << p.first << " prob " << p.second << std::endl;
        }
    }
}
```

## 常见问题

- Reshape 断言 `total(srcShape) == maskTotal`：保持输入尺寸与导出模型一致（320x320x3），确保输入是 3 通道；除非模型支持动态尺寸，否则不要随意更改 `--size`。
- 读图失败：检查路径是否存在；`inferenceFromPath` 读图失败会抛异常。
- 动态尺寸：只有在模型导出支持动态输入时，才用非 320 的尺寸调用 `setInputSize` 或 `--size`。 
