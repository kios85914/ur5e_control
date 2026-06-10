import cv2
import numpy as np
import pyrealsense2 as rs

# 配置和開始 RealSense 流
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

# 相機的內部參數
frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()
intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
fx = intrinsics.fx  # 焦距（像素）
fy = intrinsics.fy
print(fx, fy)
optical_center_x = 320  # 光心 x 坐標（像素）
optical_center_y = 240

# 截取區域的位置
x_offset = 100
y_offset = 150

real_offset_x = 0.0
real_offset_y = -0.325

# 物體到相機的距離（米）
distance = 0.63

while True:
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())

    # 截取區域
    cropped_image = color_image[y_offset : y_offset + 300, x_offset : x_offset + 457]

    # 轉換到 HSV 色彩空間
    hsv = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)

    # 定義綠色的範圍
    lower_green = np.array([35, 100, 50])
    upper_green = np.array([85, 255, 255])

    # 選取綠色區域
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    green_pixel_area = cv2.countNonZero(green_mask)
    print("green_pixel_area",green_pixel_area)

    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # 找到綠點的輪廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # 計算中心點
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # 換算成實際座標
            x_real = (
                -((cx + x_offset - optical_center_x) * distance / fx) + real_offset_x
            )
            y_real = (cy + y_offset - optical_center_y) * distance / fy + real_offset_y

            # 在原始圖像上標記中心點
            cv2.circle(cropped_image, (cx, cy), 5, (255, 0, 0), -1)
            print(x_real, y_real)

    # 顯示圖像
    cv2.imshow("Green Object Detection", cropped_image)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
pipeline.stop()
