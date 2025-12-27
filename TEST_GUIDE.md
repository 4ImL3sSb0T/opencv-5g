# 循迹代码移植测试说明

## ✅ 任务完成情况

**所有任务已完成！**

### 完成的工作

1. ✅ 移植了 pi_linux/image_Q.cpp 中的循迹代码到 trace.cpp
2. ✅ 实现了原始边界搜索算法（Edge_Search_Mid_Original）
3. ✅ 实现了改进的多候选边界搜索算法（Edge_Search_Mid_Improved）
4. ✅ 添加了模式切换功能（按键 E 切换算法）
5. ✅ 修复了 Windows 运行时错误（添加边界检查，防止数组越界）
6. ✅ 添加了完整的可视化调试功能

---

## 🎯 核心改进说明

### 原始算法 vs 改进算法 vs 霍夫跟踪算法

**原始算法（edgeSearchMode = 0）** - src/trace.cpp:315-395
- 从中点向左右搜索边界
- 找到第一个符合条件的边界就立即停止（break）
- **问题**：容易被杂线干扰，在直道与弯道交汇处会跑偏

**改进算法（edgeSearchMode = 1）** - src/trace.cpp:405-505
```cpp
std::vector<int16> left_candidates;   // 收集所有左边界候选点
std::vector<int16> right_candidates;  // 收集所有右边界候选点

// 不再 break，而是 continue 收集所有候选点
left_candidates.push_back(j);
continue;  // 继续搜索更左边的边界

// 从候选点中选择最优边界
int16 best_left = *std::min_element(left_candidates.begin(), left_candidates.end());
int16 best_right = *std::max_element(right_candidates.begin(), right_candidates.end());
```
- **优势**：能过滤掉中间的杂线，选择真正的赛道边界

**霍夫直线跟踪算法（edgeSearchMode = 2）** - src/trace.cpp:198-277 ⭐ 新算法！
```cpp
// 核心思想：基于霍夫直线检测，跟踪帧间边界线
struct TrackedLine {
    cv::Vec4i line;       // 直线段
    float slope;          // 斜率
    cv::Point2f midpoint; // 中点
    float length;         // 长度
    int age;              // 跟踪帧数
    float confidence;     // 置信度
};

// 相似度计算：加权组合
similarity = 0.4 * slope_similarity + 0.4 * position_similarity + 0.2 * length_ratio

// 每帧匹配：找到与上一帧边界线最相似的新直线
int bestIdx = findBestMatchingLine(lines, previousLine, isLeftBoundary);
```
- **核心优势**：
  1. 直接在霍夫直线层面工作，无需逐像素扫描
  2. 利用斜率和位置的时间连续性，提高抗干扰能力
  3. 置信度跟踪：匹配成功则提升，失败则降低
  4. 角度过滤：左边界 -75° 到 -25°，右边界 25° 到 75°
- **适用场景**：有杂线干扰、需要稳定跟踪的复杂场景

### 三种算法对比总结

| 算法 | 工作原理 | 优势 | 劣势 | 适用场景 |
|------|---------|------|------|---------|
| **原始算法** | 逐像素扫描，找到第一个边界就停 | 简单快速 | 易被杂线误导 | 简单场景、无杂线 |
| **改进算法** | 收集所有候选点，选择最外侧 | 能过滤中间杂线 | 仍基于像素，受噪声影响 | 中等复杂场景 |
| **霍夫跟踪** | 基于直线跟踪，相似度匹配 | 抗干扰强，时间连续性好 | 需要霍夫检测先成功 | 复杂场景、多杂线 |

### Windows 运行时错误修复

**原始代码的问题**（pi_linux/image_Q.cpp:209-231）：
```cpp
// ❌ 可能越界
if ((data.at<uchar>(i, j-8) > 100))  // 当 j < 8 时越界
if ((data.at<uchar>(i, j+8) > 100))  // 当 j > COL-9 时越界
```

**修复后的代码**（src/trace.cpp:94,118）：
```cpp
// ✅ 添加边界检查
if (j < 8) break;       // 确保 j-8 不会越界
if (j > COL - 9) break; // 确保 j+8 不会越界
```

---

## 🛠️ 编译和测试

### 编译命令

```bash
# 方法1：清理旧构建并重新配置（推荐）
rd /s /q build
mkdir build
cd build
cmake .. -G "Visual Studio 17 2022" -A x64
cmake --build . --target trace --config Release

# 方法2：直接使用现有构建系统
cd build
cmake --build . --target trace --config Release
```

### 运行测试

**测试视频文件：**
```bash
# 测试左转视频（推荐先测试这个）
.\build\Release\trace.exe -v "img\left - Trim.mp4"

# 测试右转视频
.\build\Release\trace.exe -v "img\right - Trim.mp4"

# 测试完整引导视频
.\build\Release\trace.exe -v "img\guided_full.mp4"

# 测试车库停车视频
.\build\Release\trace.exe -v "img\garage_hd_left.mp4"
```

**测试摄像头：**
```bash
# 使用默认摄像头
.\build\Release\trace.exe

# 使用指定摄像头
.\build\Release\trace.exe -c 0
```

---

## ⌨️ 交互式按键控制

| 按键 | 功能 | 说明 |
|------|------|------|
| **M** | 切换预处理模式 | 0: 顶帽+OTSU<br>1: 自适应阈值<br>2: Shadow算法 |
| **E** | **切换边界搜索模式** ⭐ | 0: 原始算法（找到第一个就停）<br>1: **改进算法（多候选边界）**<br>2: **霍夫直线跟踪算法（新！）** |
| **空格** | 暂停/继续 | 暂停视频播放，方便观察细节 |
| **N** | 单帧前进 | 在暂停状态下逐帧查看 |
| **Q** | 退出程序 | |

---

## 📊 可视化窗口说明

程序运行时会打开多个调试窗口：

1. **cropped** - 裁剪后的原始图像
2. **binary** - 二值化后的图像
3. **canny** - Canny 边缘检测结果
4. **hough_filtered** - Hough 直线检测和角度过滤结果
5. **track_result** - 🌟 **循迹结果可视化**（最重要！）
   - **绿色圆点**：左边界点
   - **蓝色圆点**：右边界点
   - **红色圆点**：中线点
   - **粗绿线**：霍夫跟踪的左边界线（仅模式2）
   - **粗蓝线**：霍夫跟踪的右边界线（仅模式2）
   - **L:xx / R:xx**：左右边界置信度（仅模式2）
   - 左上角文字：当前边界搜索模式和预处理模式
6. **Control Panel** - FPS 控制滑条

---

## 🧪 测试重点

### 1. 对比三种算法的效果（核心测试）

**建议测试流程**：

```bash
# 1. 启动程序
.\build\Release\trace.exe -v "img\left - Trim.mp4"

# 2. 按空格暂停，找到直道与弯道交汇处的帧

# 3. 按 E 键，使用原始算法（edgeSearchMode = 0）
#    观察 track_result 窗口中的边界和中线

# 4. 再按 E 键，切换到改进算法（edgeSearchMode = 1）
#    对比同一帧的效果

# 5. 再按 E 键，切换到霍夫直线跟踪算法（edgeSearchMode = 2）
#    观察跟踪线的稳定性和置信度

# 6. 按 N 键单帧前进，继续观察后续帧的表现
```

**预期结果**：
- **原始算法**：可能会被中间的杂线误导，导致中线偏移
- **改进算法**：能过滤杂线，选择最外侧的真实边界，中线更稳定
- **霍夫跟踪算法**：基于帧间跟踪，通过相似度匹配选择边界线，抗杂线干扰能力更强

### 2. 验证不同预处理模式

按 **M** 键切换预处理模式，观察不同场景下的表现：

| 模式 | 适用场景 | 特点 |
|------|----------|------|
| 0 - 顶帽+OTSU | 光照均匀 | 自动阈值，参数少 |
| 1 - 自适应阈值 | 光照不均 | 局部自适应 |
| 2 - Shadow算法 | 复杂场景 | 强滤波，更严格的角度过滤 |
| 3 - HSV阈值 | 颜色分割 | 需手动调整滑条 |

### 3. 验证 Windows 运行时错误修复

**测试方法**：
```bash
# 长时间运行测试
.\build\Release\trace.exe -v "img\left - Trim.mp4"
# 让程序循环播放多次，观察是否崩溃
```

**预期**：程序稳定运行，不会出现数组越界错误或崩溃

---

## 📈 性能观察

### 关键指标

在 **track_result** 窗口中观察：

1. **边界检测准确性**：绿色（左）和蓝色（右）圆点是否紧贴赛道边缘
2. **中线稳定性**：红色圆点连成的中线是否平滑，无剧烈抖动
3. **抗干扰能力**：在有杂线的情况下，改进算法是否能正确识别边界
4. **补线效果**：在丢线（边界不连续）时，补线是否合理

### 典型测试场景

| 场景 | 测试视频 | 观察重点 |
|------|----------|----------|
| 直道 | guided_full.mp4 | 边界平行，中线居中 |
| 弯道 | left/right - Trim.mp4 | 边界曲线平滑 |
| 交叉路口 | left - Trim.mp4 | 抗杂线干扰能力 ⭐ |
| 车库停车 | garage_hd_*.mp4 | 横向线检测 |

---

## 🐛 问题排查

### 问题1：看不到边界线

**可能原因**：预处理模式不适合当前视频
**解决方法**：按 **M** 键切换预处理模式，尝试模式 2（Shadow算法）

### 问题2：边界检测不准确，中线偏移

**可能原因**：当前使用的是原始算法
**解决方法**：按 **E** 键确保切换到改进算法（窗口左上角显示 "Edge Mode: Improved"）

### 问题3：HSV 模式无效果

**可能原因**：HSV 参数未正确设置
**解决方法**：
1. 在 "HSV Tuning" 窗口调整滑条
2. 推荐白色范围：H(0-180), S(0-60), V(160-255)

### 问题4：程序崩溃

**检查点**：
1. 确认 OpenCV 和 spdlog 已正确安装
2. 确认视频文件路径正确（注意路径中的空格需要用引号括起来）
3. 检查是否有编译警告

---

## 📝 命名映射表（方便理解代码）

| 原代码拼音/英文 | 标准英文名称 | 中文含义 |
|----------------|-------------|----------|
| `Earge_Search_Mid` | `Edge_Search_Mid` | 边界搜索 |
| `Left_Add_Line` | `Left_Add_Line` | 左边界补线 |
| `Right_Add_Line` | `Right_Add_Line` | 右边界补线 |
| `Left_Add_Flag` | `Left_Add_Flag` | 左边界补线标志 |
| `Mid_Line` | `Mid_Line` | 中线 |
| `Road_Width_Real` | `Road_Width_Real` | 实际赛道宽度 |
| `Road_Width_Add` | `Road_Width_Add` | 补线赛道宽度 |
| `TUxiang_Init` | `Image_Init` | 图像初始化 |
| `banma` | `zebra_crossing` | 斑马线 |
| `bizhang` | `obstacle` | 避障 |

---

## 📚 代码结构说明

### 主要函数（src/trace.cpp）

```cpp
// 霍夫直线特征提取 - 第64-83行
TrackedLine extractLineFeatures(const cv::Vec4i& line);

// 直线相似度计算 - 第91-109行
float calculateLineSimilarity(const TrackedLine& line1, const TrackedLine& line2);

// 最佳匹配直线查找 - 第118-148行
int findBestMatchingLine(const std::vector<cv::Vec4i>& lines,
                        const TrackedLine& previousLine,
                        bool isLeftBoundary);

// 边界线初始化 - 第154-191行
void initializeBoundaryLines(const std::vector<cv::Vec4i>& lines);

// 边界搜索（霍夫跟踪算法）- 第198-277行 ⭐ 新算法！
void Edge_Search_HoughTracking(const std::vector<cv::Vec4i>& lines, int imageHeight);

// 边界搜索（原始算法）- 第315-395行
void Edge_Search_Mid_Original(int16 i, const cv::Mat& data, int16 Mid,
                               int16 Left_Min, int16 Right_Max);

// 边界搜索（改进算法）- 第405-505行
void Edge_Search_Mid_Improved(int16 i, const cv::Mat& data, int16 Mid,
                               int16 Left_Min, int16 Right_Max);

// 图像处理主函数 - 第537-576行
int Image_Handle(const cv::Mat& data,
                const std::vector<cv::Vec4i>& houghLines = std::vector<cv::Vec4i>(),
                int imageHeight = ROW);

// 绘制循迹结果 - 第583-660行
void Draw_Track_Result(cv::Mat& display, const cv::Mat& data);

// 中线线性插值 - 第510-517行
void LinearInterpolation();

// 首行处理 - 第297-305行
int16 First_Line_Handle();

// 限幅保护 - 第286-291行
int16 Limit_Protect(int16 num, int32 min, int32 max);
```

### 关键数据结构

**霍夫跟踪相关（第33-45行）**
```cpp
struct TrackedLine {
    cv::Vec4i line;           // 直线段 (x1, y1, x2, y2)
    float slope;              // 斜率
    cv::Point2f midpoint;     // 中点
    float length;             // 长度
    int age;                  // 跟踪帧数
    float confidence;         // 置信度
};

TrackedLine leftBoundaryLine;   // 左边界线
TrackedLine rightBoundaryLine;  // 右边界线
bool leftLineInitialized = false;
bool rightLineInitialized = false;
```

**传统循迹相关（第48-56行）**

```cpp
int16  Left_Line[ROW + 2];         // 实际左边界
int16  Right_Line[ROW + 2];        // 实际右边界
int16  Mid_Line[ROW + 2];          // 赛道中线
int16  Left_Add_Line[ROW + 2];     // 左边界补线
int16  Right_Add_Line[ROW + 2];    // 右边界补线
int16  Left_Add_Flag[ROW + 2];     // 左边界补线标志（1=需要补线，0=已找到边界）
int16  Right_Add_Flag[ROW + 2];    // 右边界补线标志
```

---

## 🎯 建议测试顺序

1. **基础功能测试**（5分钟）
   ```bash
   .\build\Release\trace.exe -v "img\left - Trim.mp4"
   # 确保程序正常运行，能看到 track_result 窗口
   ```

2. **算法对比测试**（15分钟）⭐
   ```bash
   # 找到有杂线的帧，按空格暂停
   # 按 E 键循环切换三种算法（原始 → 改进 → 霍夫跟踪 → 原始...）
   # 对比同一帧在不同算法下的效果
   # 重点观察：边界线的准确性、中线的稳定性、对杂线的抗干扰能力
   ```

3. **预处理模式测试**（10分钟）
   ```bash
   # 按 M 键切换不同预处理模式
   # 观察哪种模式效果最好
   ```

4. **稳定性测试**（5分钟）
   ```bash
   # 让程序循环播放，确保不崩溃
   ```

---

## 🔬 霍夫直线跟踪算法详解（模式2）

### 算法原理

传统的像素级边界搜索（模式0和1）容易受到噪声和杂线的干扰。霍夫直线跟踪算法采用了完全不同的思路：

1. **直接在直线层面工作**：利用已经检测到的霍夫直线，而不是逐像素扫描
2. **帧间连续性跟踪**：假设相邻帧的边界线特征（斜率、位置）变化平缓
3. **相似度匹配**：通过计算新直线与历史边界线的相似度来选择最佳匹配

### 核心组件

**1. 直线特征提取（extractLineFeatures）**
```cpp
TrackedLine {
    line:     霍夫直线段坐标 (x1, y1, x2, y2)
    slope:    斜率 = dy/dx
    midpoint: 中点坐标 = ((x1+x2)/2, (y1+y2)/2)
    length:   长度 = sqrt(dx² + dy²)
    age:      已跟踪的帧数
    confidence: 置信度 [0.0, 1.0]
}
```

**2. 相似度计算（calculateLineSimilarity）**

加权组合三个维度的相似度：

- **斜率相似度**（权重 40%）
  ```
  slope_similarity = exp(-|slope1 - slope2| * 0.1)
  ```
  使用指数衰减，斜率差异越大，相似度越低

- **位置相似度**（权重 40%）
  ```
  distance = sqrt((x1-x2)² + (y1-y2)²)
  position_similarity = exp(-distance / 50.0)
  ```
  中点距离超过50像素时相似度显著下降

- **长度相似度**（权重 20%）
  ```
  length_ratio = min(len1, len2) / max(len1, len2)
  ```
  长度比越接近1，相似度越高

**3. 最佳匹配查找（findBestMatchingLine）**

对每条新检测到的霍夫直线：
1. 计算其角度，判断是否符合左/右边界的角度范围
   - 左边界：-75° 到 -25°
   - 右边界：25° 到 75°
2. 计算与上一帧边界线的相似度得分
3. 选择相似度最高（得分最低）的直线作为新的边界线

**4. 置信度管理**

- **匹配成功**：`confidence = min(1.0, confidence + 0.1)` → 置信度提升
- **匹配失败**：`confidence *= 0.9` → 置信度衰减
- 置信度反映了跟踪的稳定性，可用于判断是否需要重新初始化

### 可视化说明

在 `track_result` 窗口中，模式2会显示：

| 元素 | 颜色/样式 | 含义 |
|------|----------|------|
| 粗绿线 | 绿色，3像素宽 | 当前跟踪的左边界霍夫直线 |
| 粗蓝线 | 蓝色，3像素宽 | 当前跟踪的右边界霍夫直线 |
| L:0.85 | 绿色文字 | 左边界置信度（0.85 = 85%可信） |
| R:0.92 | 蓝色文字 | 右边界置信度（0.92 = 92%可信） |
| 绿色小圆点 | 2像素 | 左边界在每行的投影点 |
| 蓝色小圆点 | 2像素 | 右边界在每行的投影点 |
| 红色小圆点 | 2像素 | 计算出的中线点 |

### 测试要点

**场景1：直道行驶**
- 预期：左右边界线应该相对平行
- 置信度：应该保持在 0.9 以上
- 中线：应该接近图像中心

**场景2：弯道**
- 预期：边界线斜率平滑变化，不会突变
- 置信度：可能略有下降但应保持 > 0.7
- 中线：应该偏向弯道内侧

**场景3：杂线干扰**
- 预期：霍夫跟踪应该"粘"在真实边界上，忽略杂线
- 对比：与模式0/1的表现，模式2应该更稳定
- 观察：置信度是否因杂线而下降

**场景4：丢线恢复**
- 预期：短暂丢线后（如路口），能重新找回边界
- 观察：置信度下降后能否重新上升
- 调试：如果长时间丢线，可能需要调整相似度权重

### 参数调优建议

如果发现跟踪效果不理想，可以调整以下参数（在 trace.cpp 中）：

**1. 相似度权重**（第106行）
```cpp
float similarity = 0.4f * slopeSimilarity + 0.4f * positionSimilarity + 0.2f * lengthRatio;
```
- 增加斜率权重：对斜率变化更敏感（适合弯道多的场景）
- 增加位置权重：对位置变化更敏感（适合直道场景）

**2. 角度过滤范围**（第131-136行）
```cpp
// 左边界：-75° 到 -25° （当前设置）
// 右边界：25° 到 75°
```
- 缩小范围：更严格，减少误匹配，但可能在急弯时丢线
- 扩大范围：更宽松，能应对更大角度变化，但可能匹配到错误的线

**3. 置信度调整速度**（第225、228、236、239行）
```cpp
confidence + 0.1   // 成功时提升幅度
confidence * 0.9   // 失败时衰减系数
```
- 提高提升幅度：快速建立信任，但可能对误匹配过于乐观
- 降低衰减系数：失败后保持更久，但可能错误跟踪时间过长

### 常见问题排查

**Q1: 置信度一直很低（< 0.5）**
- 可能原因：霍夫直线检测质量不好
- 解决方法：调整预处理模式（按M键），尝试模式2（Shadow算法）

**Q2: 边界线频繁跳变**
- 可能原因：相似度计算对位置太敏感
- 解决方法：降低位置相似度权重，增加斜率权重

**Q3: 边界线"粘"在杂线上**
- 可能原因：角度过滤范围太宽，或初始化时选错了线
- 解决方法：重启程序，或缩小角度过滤范围

**Q4: 急弯时丢线**
- 可能原因：角度过滤范围太窄
- 解决方法：扩大角度过滤范围，例如左边界改为 -80° 到 -20°

---

## ✨ 总结

**任务已全部完成！** ✨

本次开发工作：
1. ✅ 完整移植了循迹边界搜索代码
2. ✅ 实现了三种算法供对比测试
3. ✅ 修复了 Windows 平台的运行时错误
4. ✅ 提供了完善的可视化和调试工具
5. ✅ 添加了灵活的模式切换功能
6. ✅ **新增霍夫直线跟踪算法，基于相似度匹配实现帧间连续跟踪**

**核心改进**：
- **模式1**：通过收集所有候选边界并选择最优解（min/max），有效解决了杂线干扰问题
- **模式2**：基于霍夫直线的帧间跟踪，利用斜率和位置的时间连续性，进一步提升抗干扰能力

**重要提示**：
- 测试时重点关注 **track_result** 窗口
- 使用 **E** 键循环切换三种算法的效果
- 在有杂线的场景下，模式2（霍夫跟踪）应该表现最好
- 观察置信度数值，判断跟踪质量

祝测试顺利！🎉

---

**如有问题，请检查：**
1. CMakeLists.txt 第1行已改为 `cmake_minimum_required(VERSION 3.10)`
2. src/trace.cpp 包含完整的循迹代码
3. 编译时没有错误或警告
