import cv2
import numpy as np
import math

def empty(a):
    pass

def init_trackbars():
    cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Trackbars", 600, 600)
    
    # HSV 颜色阈值
    cv2.createTrackbar("H Min", "Trackbars", 0, 179, empty)
    cv2.createTrackbar("H Max", "Trackbars", 179, 179, empty)
    cv2.createTrackbar("S Min", "Trackbars", 0, 255, empty)
    cv2.createTrackbar("S Max", "Trackbars", 255, 255, empty)
    # 因为发光体比较亮，V(亮度)值通常较高
    cv2.createTrackbar("V Min", "Trackbars", 200, 255, empty)
    cv2.createTrackbar("V Max", "Trackbars", 255, 255, empty)
    
    # 形态学操作
    cv2.createTrackbar("Close Kernel", "Trackbars", 10, 50, empty)
    
    # 轮廓属性过滤
    cv2.createTrackbar("Min Area", "Trackbars", 50, 5000, empty)
    cv2.createTrackbar("Max Area", "Trackbars", 3000, 10000, empty)
    # 长宽比（缩放10倍以便在滑动条显示浮点数）
    cv2.createTrackbar("Min Aspect (x10)", "Trackbars", 15, 100, empty)  # 默认 1.5
    cv2.createTrackbar("Max Aspect (x10)", "Trackbars", 100, 500, empty) # 默认 10.0
    
    # 配对条件过滤
    cv2.createTrackbar("Max Angle Diff", "Trackbars", 15, 90, empty)
    cv2.createTrackbar("Min Dist Ratio (x10)", "Trackbars", 10, 100, empty) 
    cv2.createTrackbar("Max Dist Ratio (x10)", "Trackbars", 60, 150, empty)

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
    h_min = cv2.getTrackbarPos("H Min", "Trackbars")
    h_max = cv2.getTrackbarPos("H Max", "Trackbars")
    s_min = cv2.getTrackbarPos("S Min", "Trackbars")
    s_max = cv2.getTrackbarPos("S Max", "Trackbars")
    v_min = cv2.getTrackbarPos("V Min", "Trackbars")
    v_max = cv2.getTrackbarPos("V Max", "Trackbars")
    
    k_size = cv2.getTrackbarPos("Close Kernel", "Trackbars")
    k_size = max(1, k_size) # 避免为0
    
    min_area = cv2.getTrackbarPos("Min Area", "Trackbars")
    max_area = cv2.getTrackbarPos("Max Area", "Trackbars")
    min_aspect = cv2.getTrackbarPos("Min Aspect (x10)", "Trackbars") / 10.0
    max_aspect = cv2.getTrackbarPos("Max Aspect (x10)", "Trackbars") / 10.0
    
    max_angle_diff = cv2.getTrackbarPos("Max Angle Diff", "Trackbars")
    min_dist_ratio = cv2.getTrackbarPos("Min Dist Ratio (x10)", "Trackbars") / 10.0
    max_dist_ratio = cv2.getTrackbarPos("Max Dist Ratio (x10)", "Trackbars") / 10.0
    
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

    return mask_closed, result_img

def main():
    # 默认调用本地摄像头，也可替换成测试视频路径，如 cap = cv2.VideoCapture("test.mp4")
    cap = cv2.VideoCapture(0)
    init_trackbars()
    
    print("按下 'ESC' 退出程序。")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            # 如果是视频播放结束，可添加循环播放逻辑或break
            break
            
        # 根据情况可能需要降低分辨率以便查看
        # frame = cv2.resize(frame, (640, 480))
        
        mask, result = process_frame(frame)
        
        cv2.imshow("Original", frame)
        cv2.imshow("Mask", mask)
        cv2.imshow("Detection Result", result)
        
        key = cv2.waitKey(30) & 0xFF
        if key == 27:
            break
            
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()