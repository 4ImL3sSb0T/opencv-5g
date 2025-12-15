# ONNX 模型推理问题排查指南

当使用训练集图片作为输入时，模型没有有用的输出？这通常是**预处理不匹配**导致的。

## 🔍 快速诊断步骤

### 1. 运行诊断脚本（推荐）

```bash
# 分析你的训练图像预处理
python script/debug_onnx_preprocess.py img/your_training_image.jpg

# 如果模型输入尺寸不是 320x320，指定正确尺寸
python script/debug_onnx_preprocess.py img/your_training_image.jpg --size 224x224
```

这个脚本会：
- 显示原始图像的统计信息
- 模拟当前 C++ 代码的预处理过程
- 模拟常见的 PyTorch/TensorFlow 预处理方式
- 对比不同预处理方法的输出

### 2. 运行 C++ 测试程序（已启用详细输出）

```bash
# 重新编译
cd build
cmake --build .

# 测试单张图片（会显示详细的预处理信息）
./onnx_test model/your_model.onnx img/test.jpg

# 测试目录中的所有图片
./onnx_test model/your_model.onnx img/

# 指定输入尺寸
./onnx_test model/your_model.onnx img/test.jpg --size 224x224
```

程序现在会输出：
```
[DEBUG] ========== 预处理详细信息 ==========
[DEBUG] 1. 原始图像:
[DEBUG]    - 尺寸: 640x480
[DEBUG]    - 通道数: 3
[DEBUG]    - 值域: [0, 255]
[DEBUG]    - BGR均值: (123.45, 116.78, 103.92)
[DEBUG] 2. Blob (blobFromImage 输出):
[DEBUG]    - 形状: [1, 3, 320, 320]
[DEBUG]    - Blob值域: [0, 1]
[DEBUG]    - Blob各通道均值 (RGB): (0.4567, 0.3891, 0.5123)
```

## 🐛 常见问题及解决方案

### 问题 1: 通道顺序不匹配

**现象:** 模型输出全是随机值或者预测错误

**原因:** 训练时用 RGB，推理时用 BGR（或相反）

**解决方法:**

1. 检查你的训练代码，确认通道顺序:
   ```python
   # PyTorch 中，PIL.Image 和 torchvision.transforms 默认使用 RGB
   from PIL import Image
   img = Image.open('test.jpg')  # RGB

   # OpenCV 默认使用 BGR
   import cv2
   img = cv2.imread('test.jpg')  # BGR
   ```

2. 修改 `src/onnx_inference.hpp:143` 的 `swapRB` 参数:
   ```cpp
   cv::dnn::blobFromImage(aligned, blob,
                          1.0f / 255.0f,
                          input_size_,
                          cv::Scalar(0, 0, 0),
                          true,   // 如果训练用 RGB，设为 true (当前)
                                  // 如果训练用 BGR，设为 false
                          false,
                          CV_32F);
   ```

### 问题 2: 归一化方式不匹配

**现象:** 模型输出值域不对，或者全是同一个类别

**原因:** 训练时使用了 ImageNet 标准化，推理时只除以 255

**训练代码示例:**
```python
# PyTorch 常见的 ImageNet 标准化
transforms.Normalize(mean=[0.485, 0.456, 0.406],
                    std=[0.229, 0.224, 0.225])
```

**解决方法:**

修改 `src/onnx_inference.hpp` 的 `preprocessImage` 函数:

```cpp
// 当前的简单归一化（仅除以 255）
cv::dnn::blobFromImage(aligned, blob,
                       1.0f / 255.0f,      // scalefactor
                       input_size_,
                       cv::Scalar(0, 0, 0), // mean
                       true,
                       false,
                       CV_32F);

// 改为 ImageNet 标准化
// 步骤 1: 先除以 255
cv::Mat normalized;
aligned.convertTo(normalized, CV_32F, 1.0f / 255.0f);

// 步骤 2: 减均值，除标准差
cv::Scalar mean(0.485, 0.456, 0.406);  // RGB 顺序
cv::Scalar std(0.229, 0.224, 0.225);

// 注意: OpenCV 是 BGR，需要调整顺序
if (swapRB) {
    mean = cv::Scalar(0.406, 0.456, 0.485);  // BGR 顺序
    std = cv::Scalar(0.225, 0.224, 0.229);
}

// 手动标准化
cv::Mat standardized;
cv::subtract(normalized, mean, standardized);
cv::divide(standardized, std, standardized);

// 步骤 3: 转换为 blob
blob = cv::dnn::blobFromImage(standardized,
                              1.0,  // 已经标准化，不需要再缩放
                              input_size_,
                              cv::Scalar(0, 0, 0),
                              swapRB,  // 根据训练时的通道顺序设置
                              false,
                              CV_32F);
```

### 问题 3: 输入尺寸不匹配

**现象:** 模型报错或输出形状不对

**原因:** 训练时用 224x224，推理时用 320x320

**解决方法:**

```bash
# 查看模型期望的输入尺寸
# 方法 1: 使用 Netron 查看模型结构
# 访问 https://netron.app/ 并上传你的 .onnx 文件

# 方法 2: 使用 onnx 库
python -c "import onnx; model = onnx.load('model.onnx'); print(model.graph.input[0])"

# 使用正确的尺寸运行测试
./onnx_test model.onnx test.jpg --size 224x224
```

### 问题 4: 数据类型或值域问题

**现象:** 模型输出 NaN 或 Inf

**可能原因:**
- 训练时用 `[0, 1]` 范围，推理时用 `[-1, 1]`（或相反）
- 训练时用 `[0, 255]` uint8，推理时用 float32

**检查方法:**

对比诊断脚本和 C++ 程序的输出:
```
# 诊断脚本输出
Blob值域: [-2.1179, 2.6400]  # ImageNet 标准化后的范围

# C++ 程序输出
Blob值域: [0.0000, 1.0000]    # 仅除以 255 的范围

# 如果这两个不一致，说明预处理方式不对！
```

## 📋 完整的排查清单

按顺序检查以下项目：

- [ ] **输入尺寸**: 训练时用的是多少？（常见: 224x224, 320x320, 416x416）
- [ ] **通道顺序**: 训练时用的是 RGB 还是 BGR？
- [ ] **归一化方式**:
  - [ ] 仅除以 255？
  - [ ] ImageNet 标准化？
  - [ ] 其他自定义归一化？
- [ ] **数据类型**: 训练时用的数据类型和范围
- [ ] **均值减去**: 训练时是否减去了均值？
- [ ] **标准差除**: 训练时是否除以了标准差？

## 🔧 实际操作流程

### Step 1: 找到训练代码中的预处理部分

```python
# 示例: PyTorch 训练代码
transform = transforms.Compose([
    transforms.Resize((224, 224)),           # ← 输入尺寸
    transforms.ToTensor(),                    # ← 转换为 Tensor，除以 255
    transforms.Normalize(                     # ← 标准化
        mean=[0.485, 0.456, 0.406],          # ← 均值 (RGB 顺序)
        std=[0.229, 0.224, 0.225]            # ← 标准差
    )
])
```

### Step 2: 记录所有参数

| 参数 | 训练时的值 |
|------|-----------|
| 输入尺寸 | 224x224 |
| 通道顺序 | RGB (PIL.Image) |
| 归一化 | ImageNet 标准化 |
| 均值 | [0.485, 0.456, 0.406] |
| 标准差 | [0.229, 0.224, 0.225] |

### Step 3: 对比当前推理代码

运行诊断脚本:
```bash
python script/debug_onnx_preprocess.py img/train_sample.jpg --size 224x224
```

查看输出，找到与训练时匹配的预处理方式。

### Step 4: 修改 C++ 代码

根据训练时的参数，修改 `src/onnx_inference.hpp:139-145`

### Step 5: 重新编译并测试

```bash
cd build
cmake --build .
./onnx_test model.onnx img/train_sample.jpg --size 224x224
```

检查输出是否符合预期。

## 💡 额外的调试技巧

### 1. 保存预处理后的图像

在训练代码中保存预处理后的图像:
```python
import matplotlib.pyplot as plt
img = transform(Image.open('test.jpg'))
plt.imshow(img.permute(1, 2, 0))  # CHW -> HWC
plt.savefig('preprocessed.png')
```

在 C++ 代码中也保存预处理后的图像，进行对比。

### 2. 使用相同的随机图像

生成一个纯色图像（如全白或全黑），用训练和推理代码分别处理，对比结果:
```python
# Python
import numpy as np
img = np.ones((224, 224, 3), dtype=np.uint8) * 128  # 灰色图像
# 运行训练时的预处理
print(transform(Image.fromarray(img)))
```

```cpp
// C++
cv::Mat test_img(224, 224, CV_8UC3, cv::Scalar(128, 128, 128));
cv::Mat blob = model.makeBlob(test_img);
// 打印 blob 的统计信息
```

### 3. 对比模型输出

使用完全相同的图像和预处理，对比:
- Python/PyTorch 的模型输出
- C++/OpenCV DNN 的模型输出

如果输出一致，说明预处理正确；如果不一致，继续排查。

## 🎯 最常见的解决方案

**80% 的问题是以下两种之一:**

1. **通道顺序错误**: 修改 `swapRB` 参数
2. **缺少 ImageNet 标准化**: 添加均值和标准差的计算

## 📞 还是无法解决？

提供以下信息以便进一步诊断:

1. 训练代码中的 `transforms` 或数据预处理部分
2. 模型的输入输出形状（使用 Netron 查看）
3. 运行诊断脚本的完整输出
4. 运行 `./onnx_test` 的完整输出
5. 一张训练集的样本图像

## 参考资源

- [OpenCV DNN 模块文档](https://docs.opencv.org/4.x/d6/d0f/group__dnn.html)
- [blobFromImage 详细说明](https://docs.opencv.org/4.x/d6/d0f/group__dnn.html#ga29f34df9376379a603acd8df581ac8d7)
- [Netron - 可视化 ONNX 模型](https://netron.app/)
- [ONNX 官方文档](https://onnx.ai/onnx/intro/)
