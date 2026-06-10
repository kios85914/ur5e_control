import threading
import cv2
import pyrealsense2 as rs
import numpy as np
import mouse
import time
import copy


class CamCapture:
    def __init__(
        self,
    ):
        self.status = False
        self.isstop = False
        self.freq = 30
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, self.freq)
        self.pipeline.start(self.config)
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        self.fx = intrinsics.fx  # 焦距（像素）
        self.fy = intrinsics.fy
        self.distance_to_plane = 0.75
        self.optical_center_x = 320  # 光心 x 坐標（像素）
        self.optical_center_y = 240  # 光心 y 坐標（像素）
        self.origin_offset_x = 100
        self.origin_offset_y = 150
        self.real_offset_x = 0.0
        ##F offset
        # self.real_offset_y = -0.33
        ##T offset
        self.real_offset_y = -0.325
        self.location = [0, 0]
        self.pre_location = [0, 0]
        self.color_image = np.zeros([300, 460])
        self.Fall_down = False

    def only_get_once(self):
        for i in range(10):
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            cropped_image = color_image[
                self.origin_offset_y : self.origin_offset_y + 300,
                self.origin_offset_x : self.origin_offset_x + 460,
            ]
            self.color_image = cropped_image
            self.locating(cropped_image)
        print("image_shape", self.color_image)
        return self.get_pos()

    def start(self):
        # 把程式放進子執行緒，daemon=True 表示該執行緒會隨著主執行緒關閉而關閉。
        print("cam started!")
        camthread = threading.Thread(target=self.queryframe, daemon=True, args=()).start()


    def restart(self):
        self.pipeline.stop()

        # 硬體重置
        ctx = rs.context()
        devices = ctx.query_devices()
        for dev in devices:
            dev.hardware_reset()

        # 等待硬體重置完成
        time.sleep(2)

        # 重新配置和啟動管道
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, self.freq)
        self.pipeline.start(self.config)

        print("Camera restarted!")

    def stop(self):
        # 記得要設計停止無限迴圈的開關。
        self.isstop = True
        print("cam stopped!")
        cv2.destroyAllWindows()
        self.pipeline.stop()

    def getframe(self):
        # 當有需要影像時，再回傳最新的影像。
        img = self.color_image
        return img

    def get_pos(self):
        return self.location

    def queryframe(self):
        while not self.isstop:
            success, frames = self.pipeline.try_wait_for_frames(
                timeout_ms=5000
            )  # 等待 5 秒
            if not success:
                print("No frames received within timeout, retrying...")
                self.restart()  # 如果需要，可以在這裡調用重啟方法
                continue
            # Wait for a coherent pair of frames: depth and color
            # frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            cropped_image = color_image[
                self.origin_offset_y : self.origin_offset_y + 300,
                self.origin_offset_x : self.origin_offset_x + 457,
            ]
            self.color_image = cropped_image
            self.locating(cropped_image)
            self.Fall_down = self.check_fall_down(cropped_image)

    def locating(self, color_image):
        # 轉換到 HSV 色彩空間
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # # 定義綠色的範圍
        # lower_green = np.array([35, 100, 100])
        # upper_green = np.array([85, 255, 255])

        # # 選取綠色區域
        # mask = cv2.inRange(hsv, lower_green, upper_green)
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
                # 在原始圖像上標記中心點

                x_real = (
                    -(
                        (cx + self.origin_offset_x - self.optical_center_x)
                        * self.distance_to_plane
                        / self.fx
                    )
                    + self.real_offset_x
                )
                y_real = (
                    cy + self.origin_offset_y - self.optical_center_y
                ) * self.distance_to_plane / self.fy + self.real_offset_y
                # if self.location[0] == 0 and self.location[1] == 0:
                #     self.pre_location = np.array([x_real, y_real])
                #     self.location = np.array([x_real, y_real])
                # elif abs(self.pre_location[0]-x_real)<=0.3 and abs(self.pre_location[1]-y_real)<=0.3:
                #     self.pre_location = copy.deepcopy(self.location)
                #     self.location = np.array([x_real, y_real])
                # else:
                #     print("camera positioning unstable", self.pre_location, x_real, y_real)
                self.location = np.array([x_real, y_real])
                # cv2.circle(color_image, (cx, cy), 5, (255, 0, 0), -1)
                # print("inner", self.location)
        # 顯示圖像
        # cv2.imshow("Green Object Detection", color_image)
    
    def check_fall_down(self,color_image):
        # 定義綠色的範圍
        lower_green = np.array([35, 100, 50])
        upper_green = np.array([85, 255, 255])
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        green_pixel_area = cv2.countNonZero(green_mask)
        if green_pixel_area >=1800 or green_pixel_area<=50:
            return True
        elif self.location[0]<=-0.45:
            return True
        else:
            return False
