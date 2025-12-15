import numpy as np
from PIL import Image
import onnxruntime
import time
import os


# --- 1. 定义预处理函数 ---
# 保持不变，因为这部分是正确的
def preprocess_image(image_path, model_input_shape):
    # 模型期望的 W 和 H (通常是第二个和第三个维度)
    _, _, H, W = model_input_shape

    img = Image.open(image_path).convert('RGB')
    img = img.resize((W, H))

    img_data = np.asarray(img, dtype=np.float32)
    img_data = img_data / 255.0

    # 确保 mean_vec 和 std_vec 为 float32
    mean_vec = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    std_vec = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    img_data = (img_data - mean_vec) / std_vec

    img_data = img_data.transpose([2, 0, 1])
    input_tensor = np.expand_dims(img_data, axis=0)

    # 返回输入张量和原始图像，以便后处理时计算缩放比例
    return input_tensor, img.size


# --- 2. 核心后处理函数：NMS (非极大值抑制) ---
def nms_numpy(boxes, scores, iou_threshold):
    """
    使用 NumPy 实现 NMS，筛选重叠的边界框。
    """
    if len(boxes) == 0:
        return []

    # 提取坐标
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    # 计算面积
    areas = (x2 - x1) * (y2 - y1)

    # 按得分排序
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        # 计算 IOU (交并比)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h

        ovr = inter / (areas[i] + areas[order[1:]] - inter)

        # 保留 IOU 小于阈值的框
        inds = np.where(ovr <= iou_threshold)[0]
        order = order[inds + 1]

    return keep


# --- 3. YOLO 模型后处理函数 ---
def postprocess_yolo_output(output_data, original_img_size, model_input_shape, conf_thres=0.25, iou_thres=0.45):
    """
    解析 YOLOv8 格式的 ONNX 输出 (1, 6300, 7)。
    :param output_data: 模型的原始输出张量
    :param original_img_size: 原始图像尺寸 (W, H)
    :param model_input_shape: 模型输入尺寸
    """
    # 移除批次维度 (1, 6300, 7) -> (6300, 7)
    predictions = np.squeeze(output_data, axis=0)

    # 类别数 (C=7): 4 (box) + 1 (conf) + 2 (classes) => 2个类别
    num_classes = predictions.shape[1] - 5

    # a. 筛选置信度低的框
    # 找到所有预测框中的最大类别置信度
    max_scores = np.max(predictions[:, 5:5 + num_classes], axis=1)
    # 计算对象置信度 * 最大类别置信度
    conf_scores = predictions[:, 4] * max_scores

    # 筛选
    valid_mask = conf_scores > conf_thres
    predictions = predictions[valid_mask]
    conf_scores = conf_scores[valid_mask]

    if predictions.shape[0] == 0:
        return []  # 没有检测到任何对象

    # b. 坐标转换和解码
    # YOLO 默认输出是 (center_x, center_y, width, height) 格式
    boxes = predictions[:, :4]

    # 转换为 (x1, y1, x2, y2) 格式
    x_center = boxes[:, 0]
    y_center = boxes[:, 1]
    width = boxes[:, 2]
    height = boxes[:, 3]

    boxes[:, 0] = x_center - width / 2  # x1
    boxes[:, 1] = y_center - height / 2  # y1
    boxes[:, 2] = x_center + width / 2  # x2
    boxes[:, 3] = y_center + height / 2  # y2

    # c. 类别 ID
    class_ids = np.argmax(predictions[:, 5:5 + num_classes], axis=1)

    # d. NMS (非极大值抑制)
    keep_indices = nms_numpy(boxes, conf_scores, iou_thres)
    final_boxes = boxes[keep_indices]
    final_scores = conf_scores[keep_indices]
    final_classes = class_ids[keep_indices]

    # e. 缩放回原始图像尺寸（如果您的预处理没有 Letterbox）

    # 模型输入尺寸 (W_model, H_model)
    _, _, H_model, W_model = model_input_shape

    # 原始图像尺寸 (W_orig, H_orig)
    W_orig, H_orig = original_img_size

    # 计算缩放比例
    scale_x = W_orig / W_model
    scale_y = H_orig / H_model

    # 缩放坐标
    final_boxes[:, 0] *= scale_x
    final_boxes[:, 2] *= scale_x
    final_boxes[:, 1] *= scale_y
    final_boxes[:, 3] *= scale_y

    # 整理输出结果
    results = []
    for box, score, class_id in zip(final_boxes, final_scores, final_classes):
        results.append({
            'box': box.astype(int).tolist(),  # (x1, y1, x2, y2) 像素坐标
            'score': float(score),
            'class_id': int(class_id)
        })

    return results


# --- 4. ONNX 推理流程 (主逻辑修改) ---
def run_onnx_inference(onnx_path, image_path):
    if not os.path.exists(onnx_path) or not os.path.exists(image_path):
        print(f"\n错误：文件未找到。请确保 {onnx_path} 和 {image_path} 文件存在。")
        return

    try:
        ort_session = onnxruntime.InferenceSession(onnx_path)
    except Exception as e:
        print(f"加载 ONNX 模型失败: {e}")
        return

    input_name = ort_session.get_inputs()[0].name
    input_shape = ort_session.get_inputs()[0].shape
    print(f"模型期望输入形状 (Input Name: {input_name}): {input_shape}")

    # 预处理图像, 并获取原始图像尺寸
    print("正在进行图像预处理...")
    input_tensor, original_img_size = preprocess_image(image_path, input_shape)

    # 运行推理
    start_time = time.time()
    print("正在运行 ONNX 推理...")

    ort_inputs = {input_name: input_tensor}
    ort_outputs = ort_session.run(None, ort_inputs)

    end_time = time.time()
    output_data = ort_outputs[0]

    print(f"\n🎉 推理完成！耗时: {end_time - start_time:.4f} 秒")
    print(f"模型原始输出形状: {output_data.shape}")

    # ----------------------------------------------
    # 🎯 新增的 YOLO 后处理步骤
    print("\n🔍 正在进行 YOLO 后处理 (NMS)...")

    # 注意：这里假设您的模型是 320x320 输入
    # 实际项目中应动态读取 input_shape

    final_results = postprocess_yolo_output(
        output_data,
        original_img_size,
        input_shape,
        conf_thres=0.25,  # 调整置信度阈值
        iou_thres=0.45  # 调整 NMS IOU 阈值
    )

    # ----------------------------------------------

    # 打印最终结果
    if final_results:
        print(f"✨ 成功检测到 {len(final_results)} 个目标:")
        for res in final_results:
            # 假设类别 0 是 "A"，类别 1 是 "B"
            class_map = {0: "A类", 1: "B类"}
            class_name = class_map.get(res['class_id'], f"未知类别 {res['class_id']}")

            # (x1, y1, x2, y2)
            box = res['box']
            print(
                f"  - 类别: {class_name} (ID: {res['class_id']}), 得分: {res['score']:.2f}, 边界框: [{box[0]}, {box[1]}, {box[2]}, {box[3]}]")

        #
    else:
        print("🤷‍♂️ 未检测到满足阈值要求的任何目标。")


# --- 5. 运行主函数 ---
if __name__ == '__main__':
    # ⚠️ 请根据您的实际路径修改
    ONNX_MODEL_PATH = "../model/AB.onnx"
    IMAGE_FILE_PATH = "../dataset/AB/images/img_0004_20251210_162928_996.jpg"

    # 为了使代码正确运行，请确保您的 ONNX 模型输入尺寸是 320x320
    # 如果不是，请确保模型的元数据或文件名提示了正确的尺寸。

    try:
        # ⚠️ 请确保您在 run_onnx_inference 的调用中传递了正确的路径
        run_onnx_inference(ONNX_MODEL_PATH, IMAGE_FILE_PATH)
    except FileNotFoundError:
        print("\n错误：请检查路径设置，确保模型和图片文件存在。")
    except Exception as e:
        print(f"\n发生运行时错误: {e}")