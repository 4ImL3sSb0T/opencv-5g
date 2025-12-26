## image_Q.cpp 处理逻辑详解

### 1. 图像预处理与输入
- **入口**：`TUxiang_Init(cv::Mat data)`；对输入帧取下部 ROI（约车前方），缩放 0.5 得到 ~320×96 的工作尺寸。
- **滤波与边缘**：灰度 → 双边滤波（7,60,60）→ 高斯(5×5, σ=30) → Canny(30,50) → 膨胀(2×2)。
- **Hough 直线过滤**：按角度分左右两组（左线 -90~-18°, 右线 18~90°），绘制到 `line_image` 并再次膨胀，得到扫线用的二值图 `dilated_ca2`。
- **调用**：`Image_Handle22(dilated_ca2, hsv_image, dilated_ca)` 完成扫线、中线与元素检测，返回误差。

### 2. 扫线与补线（`Image_Handle22` / `Earge_Search_Mid`）
- **首行参考**：`First_Line_Handle` 采用上一帧 `Mid_Line[ROW+1]` 作为虚拟首行中心，越界则置 160。
- **逐行搜索**：从底向上隔行（步长2）调用 `Earge_Search_Mid`，以上一行中线为起点，左右各步长4像素搜索黑白跳变（<100/<100/>100）。若在极端偏移区（左>中心+50或右<中心-50）则回退/前进30像素继续找。
- **补线策略**：未找到边界则用对侧边界 ±200 像素对称填充（底部行用底行值，上方行用前两行补线）；`Left_Add_Flag/Right_Add_Flag` 标记是否补线。
- **宽度与中线**：记录 `Road_Width_Real/Add`，并设 `Mid_Line = (Left_Add_Line + Right_Add_Line)/2`。`Mid_Line_Repair` 在避障改线后重算。
- **插值**：`LinearInterpolation` 用相邻两行中线均值填补间隔，`Interpolated_Liness` 保存插值结果。

### 3. 误差计算（`error_get`）
- 对每行 `(Mid_Line[i] - 320)` 乘以行权重 `Weight_th`（下方权重大、上方小）求和，归一化后截断 [-160,160]，再除以 4 作为最终误差输出。
- 存在 `stabilize_error` 冻结保护（上一帧≥6且当前≤0则冻结为上一帧，直到恢复≥0解冻），但当前未在 `Image_Handle22` 中调用。

### 4. 元素/事件处理
- **黄色停车**：`yellow_chuli` 基于 HSV 黄色阈值生成 mask；`CAR_STOP` 在底部区域计数亮点 ≥80 置 `CAR_STOP_FLAG`。由 `yellowenable` 控制。
- **障碍物检测**：`BZ_chuli` 提取蓝色（或 TrackBar 自定义颜色），腐蚀/膨胀后找最大轮廓，面积>2 才有效，中心坐标存入 `find_XYdata`。`BZ_Imageflag` 可视化。
- **障碍物是否在赛道内**：`BZ_PANDUAN_2` 根据障碍点所在行的补线，若 `x` 落在左右补线 ±25 内则判定在赛道中。
- **避障逻辑**：`BZ_LuoJISET` 目前固定选择右避障（`zhangai_Left_or_Right = 1`），当障碍首次出现在底部（y>520）后调用 `bizhangBuxian` 将补线整体平移 `Bizhang_line_move±6`，并 `Mid_Line_Repair` 重算。`bz_con` 环形队列需多帧检测（≥2）才触发。
- **斑马线检测**：`BanMa_Find111` 在边缘图上找宽高 5–55 的矩形，要求 y、x 在赛道或限定范围内；帧队列 8 帧多数表决（≥3 视为有效），并检查前4个块的 y 离散度。命中记录 `banmaxian_Y`，外层置 `stopbanma`。
- **备用斑马线**：`BanMa_Find` 有 ROI+Canny+轮廓的备用实现，目前主调用为 `BanMa_Find111`。

### 5. 开关与调试
- TrackBar（`UI_init`）：HSV 上下限、颜色选择（`red_set/yellow_set`）、显示开关（`saidao/saidao1/banma111/banma222` 等）。
- 图像显示开关：`XUNJI_Imageflag`（循迹）、`BZ_Imageflag`/`yellow_Imageflag` 等控制可视化窗口。

### 6. 关键注意点/潜在风险
- 大量硬编码阈值（搜索步长、黑白阈 100、补线偏移 200、障碍判定 ±25 等）与缩放尺寸强耦合，变更分辨率需整体审视。
- 边界补线采用对侧对称 ±200，可能在宽度突变或首次帧未初始化时产生偏差；`avg_width` 计算未使用，仍有改进空间。
- 误差冻结逻辑未接入主流程，当前误差直接受补线和权重影响，突发跳变时可能不够平滑。
- 避障方向暂硬编码为右侧，注释提到箭头决策但未实现，存在场景适配风险。
- 全局状态跨帧复用，需保证启动/异常帧的初始值合理，以免补线/宽度计算异常。
