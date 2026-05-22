import cv2
import numpy as np
import math

# ============================================================
#  参 数 说 明
# ============================================================
# 【颜色阈值】 — 基于 HSV 颜色空间过滤出目标发光区域
#   H Min / H Max : 色相范围 (0-179)，决定目标颜色（红≈0/160-179，蓝≈100-124，绿≈40-80）
#   S Min / S Max : 饱和度范围 (0-255)，值越高颜色越纯
#   V Min / V Max : 亮度范围 (0-255)，发光体亮度通常较高（推荐 V Min≥200）
#
# 【形态学操作】— 对二值掩码做闭运算，连接断裂的灯条区域
#   Close Kernel : 闭运算卷积核大小 (1-50)，越大越容易连接远处的断裂
#
# 【轮廓过滤】— 筛选单个灯条候选
#   Min Area / Max Area      : 轮廓面积范围，过滤太小/太大的噪点
#   Min Aspect / Max Aspect  : 灯条长宽比（长度÷宽度），细长灯条约 2.0~8.0
#
# 【配对条件】— 将上下两个灯条配对为一个装甲板
#   Max Angle Diff      : 两灯条主方向最大允许角度差 (0°-90°)
#   Min/Max Dist Ratio  : 灯条间距÷平均长度，太近/太远的配对会被过滤
# ============================================================

PARAM_GUIDE = """\
========== 参数调节指南 ==========
【颜色阈值】— HSV 滤出发光区域
  H Min / H Max : 色相范围 — 决定目标颜色
    红色 ≈ H=0~10 或 H=160~180    蓝色 ≈ H=100~124    绿色 ≈ H=40~80
  S Min / S Max : 饱和度范围 — 越高颜色越纯，发光体通常饱和度较高
  V Min / V Max : 亮度范围 — 发光体亮度高，推荐 V Min≥200

【形态学操作】
  Close Kernel : 闭运算核大小 — 越大越容易连接断裂灯条（过大则粘连干扰物）

【轮廓过滤】— 单个灯条筛选
  Min/Max Area       : 轮廓像素面积 — 过滤太小噪点或太大背景
  Min/Max Aspect     : 灯条长宽比 — 细长灯条约 2.0~8.0

【配对条件】— 两个灯条配成装甲板
  Max Angle Diff     : 允许的角度差 (°) — 平行灯条角度差应接近 0°
  Min/Max Dist Ratio : 距离比 — (中心距÷平均长度)，过近则可能是同侧灯条

操作提示:
  ESC  — 退出程序
  R    — 重新加载 test.png
  调节 Trackbars 面板的滑块可实时观察检测效果
=====================================
"""


def empty(a):
    pass


def init_trackbars():
    """初始化参数调节面板"""
    WIN = "Control Panel"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1000, 900)

    # ── 1. HSV ──
    cv2.createTrackbar("--- [HSV] ---", WIN, 0, 1, empty)
    cv2.setTrackbarMin("--- [HSV] ---", WIN, 0)
    cv2.createTrackbar("H.Min", WIN, 0, 179, empty)
    cv2.createTrackbar("H.Max", WIN, 179, 179, empty)
    cv2.createTrackbar("S.Min", WIN, 0, 255, empty)
    cv2.createTrackbar("S.Max", WIN, 255, 255, empty)
    cv2.createTrackbar("V.Min", WIN, 200, 255, empty)
    cv2.createTrackbar("V.Max", WIN, 255, 255, empty)

    # ── 2. Morph ──
    cv2.createTrackbar("--- [Morph] ---", WIN, 0, 1, empty)
    cv2.setTrackbarMin("--- [Morph] ---", WIN, 0)
    cv2.createTrackbar("CloseKernel", WIN, 10, 50, empty)

    # ── 3. Contour ──
    cv2.createTrackbar("--- [Contour] ---", WIN, 0, 1, empty)
    cv2.setTrackbarMin("--- [Contour] ---", WIN, 0)
    cv2.createTrackbar("MinArea", WIN, 50, 5000, empty)
    cv2.createTrackbar("MaxArea", WIN, 3000, 10000, empty)
    cv2.createTrackbar("MinAspect(x10)", WIN, 15, 100, empty)
    cv2.createTrackbar("MaxAspect(x10)", WIN, 100, 500, empty)

    # ── 4. Pair ──
    cv2.createTrackbar("--- [Pair] ---", WIN, 0, 1, empty)
    cv2.setTrackbarMin("--- [Pair] ---", WIN, 0)
    cv2.createTrackbar("MaxAngleDiff", WIN, 15, 90, empty)
    cv2.createTrackbar("MinDistRatio(x10)", WIN, 10, 100, empty)
    cv2.createTrackbar("MaxDistRatio(x10)", WIN, 60, 150, empty)

def perform_pca(contour):
    """
    使用PCA主成分分析获取轮廓的主方向和中心
    """
    sz = len(contour)
    data_pts = np.empty((sz, 2), dtype=np.float64)
    for i in range(data_pts.shape[0]):
        data_pts[i,0] = contour[i,0,0]
        data_pts[i,1] = contour[i,0,1]
    
    # PCA计算
    mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean=None)
    
    # 提取中心点和主方向向量
    center = (int(mean[0,0]), int(mean[0,1]))
    eigenvec_primary = eigenvectors[0]   # 主成分方向（灯条延伸方向）
    eigenvec_secondary = eigenvectors[1] # 次成分方向（灯条法线方向）
    
    # 计算角度 (弧度)
    angle = math.atan2(eigenvec_primary[1], eigenvec_primary[0])
    
    return center, angle, eigenvec_primary, eigenvec_secondary, eigenvalues

def process_frame(frame):
    # 1. 获取滑动条参数
    WIN = "Control Panel"
    h_min = cv2.getTrackbarPos("H.Min", WIN)
    h_max = cv2.getTrackbarPos("H.Max", WIN)
    s_min = cv2.getTrackbarPos("S.Min", WIN)
    s_max = cv2.getTrackbarPos("S.Max", WIN)
    v_min = cv2.getTrackbarPos("V.Min", WIN)
    v_max = cv2.getTrackbarPos("V.Max", WIN)
    
    k_size = cv2.getTrackbarPos("CloseKernel", WIN)
    k_size = max(1, k_size)  # 避免为0
    
    min_area = cv2.getTrackbarPos("MinArea", WIN)
    max_area = cv2.getTrackbarPos("MaxArea", WIN)
    min_aspect = cv2.getTrackbarPos("MinAspect(x10)", WIN) / 10.0
    max_aspect = cv2.getTrackbarPos("MaxAspect(x10)", WIN) / 10.0
    
    max_angle_diff = cv2.getTrackbarPos("MaxAngleDiff", WIN)
    min_dist_ratio = cv2.getTrackbarPos("MinDistRatio(x10)", WIN) / 10.0
    max_dist_ratio = cv2.getTrackbarPos("MaxDistRatio(x10)", WIN) / 10.0
    
    # 2. HSV颜色空间转换及掩码提取
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if h_min <= h_max:
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)
    else:
        # 处理色相环绕 (如红色: 160-180 和 0-10)
        lower1 = np.array([0, s_min, v_min])
        upper1 = np.array([h_max, s_max, v_max])
        lower2 = np.array([h_min, s_min, v_min])
        upper2 = np.array([179, s_max, v_max])
        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)
        
    # 3. 闭操作将分离的灯带连接在一起
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k_size, k_size))
    mask_closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # 4. 轮廓查找与筛选
    contours, _ = cv2.findContours(mask_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    light_strips = []
    result_img = frame.copy()
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area or area > max_area:
            continue
            
        rect = cv2.minAreaRect(cnt)
        width, height = rect[1]
        if width == 0 or height == 0:
            continue
            
        length = max(width, height)
        short_side = min(width, height)
        aspect_ratio = length / short_side
        
        if aspect_ratio < min_aspect or aspect_ratio > max_aspect:
            continue
            
        # 5. PCA主成分分析优化结果
        center, angle, primary_vec, secondary_vec, eigenvals = perform_pca(cnt)
        
        light_strips.append({
            'contour': cnt,
            'rect': rect,
            'area': area,
            'length': length,
            'center': center,
            'angle': angle,
            'primary_vec': primary_vec,
            'secondary_vec': secondary_vec
        })
        
        # 绘制筛选出的灯条外接矩形
        box = cv2.boxPoints(rect)
        box = np.int32(box)
        cv2.drawContours(result_img, [box], 0, (0, 255, 0), 2)
        
        # 画PCA主方向线
        p1 = (int(center[0] - primary_vec[0] * length / 2), int(center[1] - primary_vec[1] * length / 2))
        p2 = (int(center[0] + primary_vec[0] * length / 2), int(center[1] + primary_vec[1] * length / 2))
        cv2.line(result_img, p1, p2, (0, 0, 255), 2)
        cv2.circle(result_img, center, 3, (0, 0, 255), -1)
        
    # 6. 配对上下平行灯条
    matched_pairs = []
    for i in range(len(light_strips)):
        for j in range(i + 1, len(light_strips)):
            strip1 = light_strips[i]
            strip2 = light_strips[j]
            
            # 角度差判断 (转换为度)
            angle_diff = abs(strip1['angle'] - strip2['angle']) * 180.0 / math.pi
            if angle_diff > 90:
                angle_diff = 180 - angle_diff
                
            if angle_diff > max_angle_diff:
                continue
                
            # 计算两灯条中心点距离
            dx = strip1['center'][0] - strip2['center'][0]
            dy = strip1['center'][1] - strip2['center'][1]
            center_dist = math.sqrt(dx*dx + dy*dy)
            
            # 基于长度比例进行约束筛选
            avg_length = (strip1['length'] + strip2['length']) / 2.0
            dist_ratio = center_dist / avg_length
            
            if dist_ratio < min_dist_ratio or dist_ratio > max_dist_ratio:
                continue
                
            # 7. 得到两个平行灯条之间的垂直距离
            # 取两个灯条法向向量的平均作为配对特征的垂直方向
            avg_secondary_vec = (strip1['secondary_vec'] + strip2['secondary_vec']) / 2.0
            norm = np.linalg.norm(avg_secondary_vec)
            if norm > 0:
                avg_secondary_vec /= norm
                
            center_vector = np.array([dx, dy])
            # 垂直距离 = 中心点连线向量在法线方向上的投影长度
            vertical_dist = abs(np.dot(center_vector, avg_secondary_vec))
            
            matched_pairs.append((strip1, strip2, vertical_dist))
            
            # 绘制匹配结果
            cv2.line(result_img, strip1['center'], strip2['center'], (255, 0, 0), 2)
            mid_pt = ((strip1['center'][0] + strip2['center'][0]) // 2, 
                      (strip1['center'][1] + strip2['center'][1]) // 2)
            cv2.putText(result_img, f"V-Dist: {vertical_dist:.1f}", mid_pt, 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    return mask_closed, result_img, len(light_strips), len(matched_pairs)

def main():
    """主循环 — 读取 test.png 并实时调节参数"""
    image_path = "test.png"
    frame = cv2.imread(image_path)

    if frame is None:
        print(f"❌ 错误: 无法读取 '{image_path}'，请确认文件路径正确！")
        return

    init_trackbars()
    print(PARAM_GUIDE)

    print(f"✅ 已加载: {image_path}  尺寸: {frame.shape[1]}x{frame.shape[0]}")
    print(" — [ESC] 退出    [R] 重新加载图片")

    while True:
        # 深拷贝一份，避免每次修改原图
        current_frame = frame.copy()

        mask, result, n_strips, n_pairs = process_frame(current_frame)

        # 在结果图上叠加参数信息
        WIN = "Control Panel"
        h_min = cv2.getTrackbarPos("H.Min", WIN)
        h_max = cv2.getTrackbarPos("H.Max", WIN)
        cv2.putText(result, f"H:[{h_min}-{h_max}]  Strips:{n_strips}  Pairs:{n_pairs}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.putText(result, "[ESC]=Exit  [R]=Reload", (10, result.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow("Original", frame)
        cv2.imshow("Mask", mask)
        cv2.imshow("Detection Result", result)

        key = cv2.waitKey(30) & 0xFF
        if key == 27:  # ESC
            break
        elif key == ord('r') or key == ord('R'):  # 重新加载
            frame = cv2.imread(image_path)
            if frame is not None:
                print(f"🔄 已重新加载: {image_path}")
            else:
                print(f"⚠️ 重新加载失败: {image_path}")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()