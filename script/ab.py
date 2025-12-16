import cv2
import numpy as np


def detect_blue_signs(image_path):
    # 1. 读取图像
    img = cv2.imread(image_path)
    if img is None:
        print("Image not found")
        return

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 2. 定义蓝色的HSV范围 (根据实际光照调整)
    # OpenCV中 H: 0-179, S: 0-255, V: 0-255
    lower_blue = np.array([100, 43, 46])
    upper_blue = np.array([124, 255, 255])

    # 3. 提取蓝色掩膜
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # 4. 形态学操作去噪
    kernel = np.ones((5, 5), np.uint8)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

    # 5. 查找轮廓
    contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 500:  # 过滤太小的噪点
            continue

        # 获取外接矩形
        x, y, w, h = cv2.boundingRect(cnt)

        # 提取感兴趣区域 (ROI)
        roi = img[y:y + h, x:x + w]

        # --- 识别阶段 ---

        # 转灰度并二值化，提取白色文字
        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        # 假设是蓝底白字，使用大阈值提取亮色部分(白色)
        _, roi_binary = cv2.threshold(roi_gray, 200, 255, cv2.THRESH_BINARY)

        # 再次查找ROI内部的轮廓，这次使用 RETR_CCOMP 或 RETR_TREE 来找孔洞
        char_contours, hierarchy = cv2.findContours(roi_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if hierarchy is None:
            continue

        # 统计孔洞数量
        # hierarchy结构: [Next, Previous, First_Child, Parent]
        # 如果 Parent != -1，说明它是内部轮廓（孔洞）
        holes = 0
        for i in range(len(char_contours)):
            if hierarchy[0][i][3] != -1:  # 有父轮廓，说明是洞
                holes += 1

        label = "Unknown"
        # 简单的逻辑判断
        if holes == 1:
            label = "A"
        elif holes == 2:
            label = "B"
        elif holes == 0:
            # 如果没有洞，可能是字体原因，或者是误检，可以结合 宽高比 判断
            aspect_ratio = float(w) / h
            # A通常比B稍微窄一点，或者通过Hu矩进一步判断
            label = "Maybe A/Solid"

            # 6. 绘图显示结果
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(img, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    cv2.imshow('Result', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# 使用示例
# detect_blue_signs('path_to_your_image.jpg')