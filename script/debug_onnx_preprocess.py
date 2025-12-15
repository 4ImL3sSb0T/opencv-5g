#!/usr/bin/env python3
"""
ONNX 模型预处理诊断脚本

用于诊断模型推理问题，对比训练时和推理时的预处理差异
"""

import cv2
import numpy as np
import sys
from pathlib import Path

def analyze_image(image_path):
    """分析原始图像"""
    print(f"\n{'='*60}")
    print(f"分析图像: {image_path}")
    print(f"{'='*60}")

    img = cv2.imread(image_path)
    if img is None:
        print(f"[ERROR] 无法读取图像: {image_path}")
        return None

    print(f"\n1. 原始图像信息:")
    print(f"   - 尺寸: {img.shape[1]}x{img.shape[0]}")
    print(f"   - 通道数: {img.shape[2]}")
    print(f"   - 数据类型: {img.dtype}")
    print(f"   - 值域: [{img.min()}, {img.max()}]")
    print(f"   - 均值: BGR({img[:,:,0].mean():.2f}, {img[:,:,1].mean():.2f}, {img[:,:,2].mean():.2f})")
    print(f"   - 标准差: BGR({img[:,:,0].std():.2f}, {img[:,:,1].std():.2f}, {img[:,:,2].std():.2f})")

    return img

def simulate_opencv_preprocess(img, target_size=(320, 320), swap_rb=True, scale=1.0/255.0):
    """模拟 OpenCV DNN 的 blobFromImage 预处理"""
    print(f"\n2. OpenCV DNN 预处理 (当前代码):")
    print(f"   - 目标尺寸: {target_size}")
    print(f"   - 缩放因子: {scale}")
    print(f"   - 交换RB通道: {swap_rb}")

    # Resize
    resized = cv2.resize(img, target_size)
    print(f"   - Resize后尺寸: {resized.shape}")

    # Convert to float and scale
    normalized = resized.astype(np.float32) * scale
    print(f"   - 归一化后值域: [{normalized.min():.4f}, {normalized.max():.4f}]")
    print(f"   - 归一化后均值: BGR({normalized[:,:,0].mean():.4f}, {normalized[:,:,1].mean():.4f}, {normalized[:,:,2].mean():.4f})")

    # Swap RB if needed
    if swap_rb:
        normalized = cv2.cvtColor(normalized, cv2.COLOR_BGR2RGB)
        print(f"   - 交换RB后均值: RGB({normalized[:,:,0].mean():.4f}, {normalized[:,:,1].mean():.4f}, {normalized[:,:,2].mean():.4f})")

    # Convert to blob format [1, C, H, W]
    blob = np.transpose(normalized, (2, 0, 1))  # HWC -> CHW
    blob = np.expand_dims(blob, axis=0)  # Add batch dimension

    print(f"   - Blob形状: {blob.shape}")
    print(f"   - Blob值域: [{blob.min():.4f}, {blob.max():.4f}]")
    print(f"   - Blob均值 (各通道): {blob[0, 0].mean():.4f}, {blob[0, 1].mean():.4f}, {blob[0, 2].mean():.4f}")

    return blob

def simulate_pytorch_preprocess(img, target_size=(320, 320)):
    """模拟常见的 PyTorch 预处理 (ImageNet 标准化)"""
    print(f"\n3. PyTorch 风格预处理 (ImageNet标准化):")

    # Resize
    resized = cv2.resize(img, target_size)

    # Convert BGR to RGB
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

    # Convert to float [0, 1]
    normalized = rgb.astype(np.float32) / 255.0

    # ImageNet normalization
    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    std = np.array([0.229, 0.224, 0.225], dtype=np.float32)

    standardized = (normalized - mean) / std

    print(f"   - ImageNet均值: {mean}")
    print(f"   - ImageNet标准差: {std}")
    print(f"   - 标准化后值域: [{standardized.min():.4f}, {standardized.max():.4f}]")
    print(f"   - 标准化后均值: RGB({standardized[:,:,0].mean():.4f}, {standardized[:,:,1].mean():.4f}, {standardized[:,:,2].mean():.4f})")

    # Convert to blob format
    blob = np.transpose(standardized, (2, 0, 1))
    blob = np.expand_dims(blob, axis=0)

    print(f"   - Blob形状: {blob.shape}")
    print(f"   - Blob值域: [{blob.min():.4f}, {blob.max():.4f}]")

    return blob

def simulate_simple_normalize(img, target_size=(320, 320)):
    """模拟简单归一化 (仅除以255，不交换通道)"""
    print(f"\n4. 简单归一化 (仅/255，保持BGR):")

    resized = cv2.resize(img, target_size)
    normalized = resized.astype(np.float32) / 255.0

    print(f"   - 归一化后值域: [{normalized.min():.4f}, {normalized.max():.4f}]")
    print(f"   - 归一化后均值: BGR({normalized[:,:,0].mean():.4f}, {normalized[:,:,1].mean():.4f}, {normalized[:,:,2].mean():.4f})")

    blob = np.transpose(normalized, (2, 0, 1))
    blob = np.expand_dims(blob, axis=0)

    print(f"   - Blob形状: {blob.shape}")

    return blob

def print_recommendations():
    """打印排查建议"""
    print(f"\n{'='*60}")
    print("🔍 问题排查建议:")
    print(f"{'='*60}")

    print("\n【常见问题 1】通道顺序不匹配")
    print("  ❌ 训练时用 RGB，推理时用 BGR (或相反)")
    print("  ✅ 解决方法:")
    print("     - 检查训练代码，确认是用 RGB 还是 BGR")
    print("     - 修改 onnx_inference.hpp:120 的 swapRB 参数")
    print("     - 如果训练用 BGR，设置为 false")
    print("     - 如果训练用 RGB，设置为 true (当前)")

    print("\n【常见问题 2】归一化方式不匹配")
    print("  ❌ 训练时用 ImageNet 标准化，推理时只除以 255")
    print("  ✅ 解决方法:")
    print("     - 如果训练时用了 transforms.Normalize([0.485,0.456,0.406], [0.229,0.224,0.225])")
    print("     - 需要在 preprocessImage 中添加相同的标准化")
    print("     - 修改 onnx_inference.hpp:116-122 的 blobFromImage 参数")

    print("\n【常见问题 3】输入尺寸不匹配")
    print("  ❌ 训练时用 224x224，推理时用 320x320")
    print("  ✅ 解决方法:")
    print("     - 确认模型训练时的输入尺寸")
    print("     - 使用 --size 参数指定正确尺寸")
    print("     - 例如: ./onnx_test model.onnx test.jpg --size 224x224")

    print("\n【常见问题 4】数据类型或范围问题")
    print("  ❌ 训练时用 [0,1]，推理时用 [-1,1] (或相反)")
    print("  ✅ 解决方法:")
    print("     - 检查训练代码的数据预处理")
    print("     - 确保推理时的值域与训练时一致")

    print("\n【调试步骤】")
    print("  1️⃣  在训练代码中打印预处理后的图像统计信息")
    print("  2️⃣  对比上面的输出，找到匹配的预处理方式")
    print("  3️⃣  修改 onnx_inference.hpp 中的 preprocessImage 函数")
    print("  4️⃣  重新编译并测试")

    print("\n【快速测试】")
    print("  # 启用预处理调试输出")
    print("  在 main() 中添加: model.setVerbosePreprocess(true);")
    print("  ")
    print("  # 查看详细的推理输出")
    print("  ./onnx_test model.onnx test.jpg")

def main():
    if len(sys.argv) < 2:
        print("用法: python debug_onnx_preprocess.py <image_path> [--size WxH]")
        print("\n示例:")
        print("  python debug_onnx_preprocess.py img/test.jpg")
        print("  python debug_onnx_preprocess.py img/test.jpg --size 224x224")
        return

    image_path = sys.argv[1]

    # Parse size parameter
    target_size = (320, 320)
    if "--size" in sys.argv:
        idx = sys.argv.index("--size")
        if idx + 1 < len(sys.argv):
            size_str = sys.argv[idx + 1]
            w, h = map(int, size_str.split('x'))
            target_size = (w, h)

    # Analyze image
    img = analyze_image(image_path)
    if img is None:
        return

    # Simulate different preprocessing methods
    blob1 = simulate_opencv_preprocess(img, target_size, swap_rb=True, scale=1.0/255.0)
    blob2 = simulate_pytorch_preprocess(img, target_size)
    blob3 = simulate_simple_normalize(img, target_size)

    # Print recommendations
    print_recommendations()

    print("\n" + "="*60)
    print("✅ 分析完成！请对比上述输出与你的训练代码。")
    print("="*60)

if __name__ == "__main__":
    main()
