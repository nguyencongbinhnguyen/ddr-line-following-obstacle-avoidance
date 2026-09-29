#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import freenect
import time

from collections import deque
from sensor_msgs.msg import Image
from kinect_line_detector.msg import LineData


# ============================================================
# THAM SO
# ============================================================

K_SCALE_X = 0.32 / 640
K_SCALE_Y = 0.265 / 480

CENTERLINE_STEP = 8
MIN_CENTERLINE_POINTS = 15

CURVE_MEDIAN_SIZE = 5
CURVE_ALPHA = 0.20

ANG_MEDIAN_SIZE = 5
ANG_ALPHA = 0.25
ANG_DEADBAND = 1.0

ERROR_ALPHA = 0.50

HSV_H_MIN = 100
HSV_H_MAX = 130
HSV_S_MIN = 100
HSV_S_MAX = 255
HSV_V_MIN = 0
HSV_V_MAX = 255

# ============================================================
# CAC MANG CO DINH - CHI TAO 1 LAN
# ============================================================

HSV_LOWER = np.array(
    [HSV_H_MIN, HSV_S_MIN, HSV_V_MIN],
    dtype=np.uint8
)

HSV_UPPER = np.array(
    [HSV_H_MAX, HSV_S_MAX, HSV_V_MAX],
    dtype=np.uint8
)

KERNEL_3 = np.ones((3, 3), dtype=np.uint8)
KERNEL_5 = np.ones((5, 5), dtype=np.uint8)

# ============================================================
# HAM PHU
# ============================================================

def nothing(a):
    pass


def get_video():
    result = freenect.sync_get_video()

    if result is None:
        return None

    frame, _ = result

    if frame is None:
        return None

    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)


def warpImg(img, points, w, h):
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])

    matrix = cv2.getPerspectiveTransform(pts1, pts2)

    return cv2.warpPerspective(img, matrix, (w, h))


def thresholding_fixed_hsv(img):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    return cv2.inRange(
        img_hsv,
        HSV_LOWER,
        HSV_UPPER
    )


def remove_noise(img):
    img_blurred = cv2.medianBlur(img, 5)

    img_morph = cv2.morphologyEx(img_blurred, cv2.MORPH_OPEN, KERNEL_5)

    contours, _ = cv2.findContours(img_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    mask = np.zeros_like(img)

    for contour in contours:
        if cv2.contourArea(contour) > 100:
            cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)

    return mask


def cv2_to_imgmsg(cv_image, encoding="bgr8"):
    img_msg = Image()

    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = encoding
    img_msg.is_bigendian = 0

    if len(cv_image.shape) == 3:
        img_msg.step = cv_image.shape[1] * cv_image.shape[2]
    else:
        img_msg.step = cv_image.shape[1]

    img_msg.data = cv_image.tobytes()

    return img_msg


# ============================================================
# CENTERLINE + CURVATURE
# ============================================================

def calculate_curve_centerline(binary_img):
    h, w = binary_img.shape

    center_x_px = []
    center_y_px = []

    MIN_RUN_WIDTH = 3
    MAX_CENTER_JUMP = 40.0

    previous_center = None

    # ========================================================
    # TIM CENTERLINE
    # ========================================================

    for y in range(0, h, CENTERLINE_STEP):
        xs = np.where(binary_img[y, :] > 0)[0]

        if len(xs) == 0:
            continue

        split_idx = np.where(np.diff(xs) > 1)[0] + 1
        runs = np.split(xs, split_idx)

        valid_runs = []

        for run in runs:
            if len(run) >= MIN_RUN_WIDTH:
                x_left = run[0]
                x_right = run[-1]
                x_center = (x_left + x_right) / 2.0
                run_width = x_right - x_left + 1

                valid_runs.append((x_center, run_width, x_left, x_right))

        if len(valid_runs) == 0:
            continue

        if previous_center is None:
            best_run = max(valid_runs, key=lambda r: r[1])

        else:
            candidates = []

            for run in valid_runs:
                x_center = run[0]
                distance = abs(x_center - previous_center)

                if distance <= MAX_CENTER_JUMP:
                    candidates.append((distance, -run[1], run))

            if len(candidates) == 0:
                previous_center = None
                continue

            candidates.sort(key=lambda item: (item[0], item[1]))
            best_run = candidates[0][2]

        x_center = best_run[0]

        center_x_px.append(x_center)
        center_y_px.append(y)

        previous_center = x_center

    # ========================================================
    # KIEM TRA SO DIEM
    # ========================================================

    if len(center_x_px) < MIN_CENTERLINE_POINTS:
        return (
            0.0,
            np.array(center_x_px),
            np.array(center_y_px),
            None,
            False,
            0.0,
            0.0,
            0.0
        )

    center_x_px = np.array(center_x_px, dtype=np.float64)
    center_y_px = np.array(center_y_px, dtype=np.float64)

    # ========================================================
    # TACH DOAN LINE LIEN TUC
    # ========================================================

    max_gap = CENTERLINE_STEP * 1.5

    split_idx = np.where(np.diff(center_y_px) > max_gap)[0] + 1

    x_segments = np.split(center_x_px, split_idx)
    y_segments = np.split(center_y_px, split_idx)

    valid_segments = []

    for xs_seg, ys_seg in zip(x_segments, y_segments):
        if len(xs_seg) >= MIN_CENTERLINE_POINTS:
            valid_segments.append((xs_seg, ys_seg))

    if len(valid_segments) == 0:
        return (
            0.0,
            center_x_px,
            center_y_px,
            None,
            False,
            0.0,
            0.0,
            0.0
        )

    # ========================================================
    # CHON DOAN LINE TOT NHAT
    # ========================================================

    best_x = None
    best_y = None
    best_key = None

    for xs_seg, ys_seg in valid_segments:
        segment_length = len(ys_seg)
        y_bottom = np.max(ys_seg)

        key = (y_bottom, segment_length)

        if best_key is None or key > best_key:
            best_key = key
            best_x = xs_seg
            best_y = ys_seg

    center_x_px = best_x
    center_y_px = best_y

    # ========================================================
    # FIT + CURVATURE
    # ========================================================

    center_x_m = (center_x_px - w / 2.0) * K_SCALE_X
    center_y_m = center_y_px * K_SCALE_Y

    poly_coeffs_m = np.polyfit(center_y_m, center_x_m, 2)

    A = poly_coeffs_m[0]
    B = poly_coeffs_m[1]

    y_eval = np.max(center_y_m)

    dx_dy = 2.0 * A * y_eval + B
    d2x_dy2 = 2.0 * A

    denominator = (1.0 + dx_dy ** 2) ** 1.5

    if denominator > 1e-12:
        kappa = d2x_dy2 / denominator
    else:
        kappa = 0.0

    curve_raw = -kappa

    return (
        curve_raw,
        center_x_px,
        center_y_px,
        poly_coeffs_m,
        True,
        A,
        B,
        y_eval
    )


# ============================================================
# ROS NODE
# ============================================================

class KinectLineNode(object):

    def __init__(self):
        rospy.init_node("kinect_line_node")

        # ====================================================
        # PARAM
        # ====================================================

        self.show_image = rospy.get_param("~show_image", True)
        self.show_trackbars = rospy.get_param("~show_trackbars", False)
        self.publish_debug_image = rospy.get_param("~publish_debug_image", False)
        self.frame_id = rospy.get_param("~frame_id", "kinect_frame")

        rate_hz = rospy.get_param("~rate", 60)

        self.warp_points_raw = rospy.get_param("~warp_points", [0, 45, 0, 480])

        # ====================================================
        # FILTER
        # ====================================================

        self.curve_median_buffer = deque(maxlen=CURVE_MEDIAN_SIZE)
        self.curve_filtered = 0.0
        self.curve_filter_initialized = False

        self.ang_median_buffer = deque(maxlen=ANG_MEDIAN_SIZE)
        self.ang_filtered = 0.0
        self.ang_send_last = 0.0
        self.ang_filter_initialized = False
        self.ang_send_initialized = False

        self.error_filtered = 0.0
        self.error_filter_initialized = False

        # ====================================================
        # FPS
        # ====================================================

        self.fps_last_time = time.perf_counter()
        self.fps_filtered = 0.0
        self.fps_initialized = False
        self.fps_alpha = 0.20

        self.warp_matrix = None
        self.warp_size = None
        # ====================================================
        # PUBLISHER
        # ====================================================

        self.data_pub = rospy.Publisher("/kinect/line_data", LineData, queue_size=1)

        if self.publish_debug_image:
            self.image_pub = rospy.Publisher("/kinect/debug_image", Image, queue_size=1)

        # ====================================================
        # GUI
        # ====================================================

        if self.show_trackbars:
            cv2.namedWindow("Trackbars")

            cv2.createTrackbar("Width Top", "Trackbars", self.warp_points_raw[0], 650, nothing)
            cv2.createTrackbar("Height Top", "Trackbars", self.warp_points_raw[1], 480, nothing)
            cv2.createTrackbar("Width Bottom", "Trackbars", self.warp_points_raw[2], 650, nothing)
            cv2.createTrackbar("Height Bottom", "Trackbars", self.warp_points_raw[3], 480, nothing)

        if self.show_image:
            cv2.namedWindow("final image", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("final image", 640, 480)

            #cv2.namedWindow("HSV Mask", cv2.WINDOW_NORMAL)
            #cv2.resizeWindow("HSV Mask", 640, 480)

        self.rate = rospy.Rate(rate_hz)

    # ========================================================
    # WARP POINTS
    # ========================================================

    def get_warp_points(self, wT, hT):
        if self.show_trackbars:
            wTop = cv2.getTrackbarPos("Width Top", "Trackbars")
            hTop = cv2.getTrackbarPos("Height Top", "Trackbars")
            wBot = cv2.getTrackbarPos("Width Bottom", "Trackbars")
            hBot = cv2.getTrackbarPos("Height Bottom", "Trackbars")

        else:
            wTop, hTop, wBot, hBot = self.warp_points_raw

        return np.float32([
            (wTop, hTop),
            (wT - wTop, hTop),
            (wBot, hBot),
            (wT - wBot, hBot)
        ])

    # ========================================================
    # RESET FILTER
    # ========================================================

    def reset_filters(self):
        self.ang_median_buffer.clear()
        self.ang_filtered = 0.0
        self.ang_send_last = 0.0
        self.ang_filter_initialized = False
        self.ang_send_initialized = False

        self.error_filtered = 0.0
        self.error_filter_initialized = False

        self.curve_median_buffer.clear()
        self.curve_filtered = 0.0
        self.curve_filter_initialized = False

    # ========================================================
    # MAIN LOOP
    # ========================================================

    def spin(self):

        while not rospy.is_shutdown():

            # =================================================
            # CAMERA
            # =================================================

            image = get_video()

            if image is None:
                rospy.logwarn_throttle(1.0, "Khong doc duoc frame Kinect")
                self.rate.sleep()
                continue

            # =================================================
            # FPS
            # =================================================

            fps_now = time.perf_counter()
            fps_dt = fps_now - self.fps_last_time
            self.fps_last_time = fps_now

            if fps_dt > 1e-6:
                fps_raw = 1.0 / fps_dt

                if not self.fps_initialized:
                    self.fps_filtered = fps_raw
                    self.fps_initialized = True
                else:
                    self.fps_filtered = self.fps_alpha * fps_raw + (1.0 - self.fps_alpha) * self.fps_filtered

            height, width, _ = image.shape

            # =================================================
            # WARP + HSV
            # =================================================

            #points = self.get_warp_points(width, height)
            #imgWarp = warpImg(image, points, width, height)
            if self.warp_matrix is None or self.warp_size != (width, height):
                points = self.get_warp_points(width, height)

                pts2 = np.float32([
                    [0, 0],
                    [width, 0],
                    [0, height],
                    [width, height]
                ])

                self.warp_matrix = cv2.getPerspectiveTransform(points, pts2)
                self.warp_size = (width, height)

            imgWarp = cv2.warpPerspective(
                image,
                self.warp_matrix,
                (width, height)
            )

            full_line = thresholding_fixed_hsv(imgWarp)
            full_line = remove_noise(full_line)

            
            full_line = cv2.erode(full_line, KERNEL_3, iterations=1)
            full_line = cv2.dilate(full_line, KERNEL_3, iterations=2)

            # =================================================
            # CURVE
            # =================================================

            (
                curve_raw,
                center_x,
                center_y,
                poly_coeffs_m,
                curve_valid,
                A_m,
                B_m,
                y_eval_m
            ) = calculate_curve_centerline(full_line)

            if curve_valid:
                self.curve_median_buffer.append(curve_raw)

                curve_median = float(np.median(self.curve_median_buffer))

                if not self.curve_filter_initialized:
                    self.curve_filtered = curve_median
                    self.curve_filter_initialized = True
                else:
                    self.curve_filtered = CURVE_ALPHA * curve_median + (1.0 - CURVE_ALPHA) * self.curve_filtered

            # =================================================
            # VE CENTERLINE
            # =================================================
            '''
            if curve_valid and poly_coeffs_m is not None and len(center_y) > 0:
                A, B, C = poly_coeffs_m

                y_start = int(np.min(center_y))
                y_end = int(np.max(center_y))

                previous_point = None

                for y_px in range(y_start, y_end + 1, 4):
                    y_m = y_px * K_SCALE_Y
                    x_m = A * y_m ** 2 + B * y_m + C
                    x_px = int(x_m / K_SCALE_X + width / 2.0)

                    if 0 <= x_px < width:
                        current_point = (x_px, y_px)

                        if previous_point is not None:
                            cv2.line(imgWarp, previous_point, current_point, (0, 0, 255), 2)

                        previous_point = current_point
            '''
            # =================================================
            # ANG + ERROR
            # =================================================

            # =================================================
            # ROI CHO ERROR - GIU O NUA DUOI ANH
            # =================================================
            bottom_half = imgWarp[height // 2:, :]

            Greenline_err = full_line[height // 2:, :].copy()

            Greenline_err = cv2.erode(
                Greenline_err,
                KERNEL_3,
                iterations=2
            )

            Greenline_err = cv2.dilate(
                Greenline_err,
                KERNEL_3,
                iterations=4
            )

            contours_err, _ = cv2.findContours(
                Greenline_err.copy(),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )


            # =================================================
            # ROI CHO ANG - DUA LEN GIUA ANH
            # =================================================

            Greenline_ang = full_line[height // 2:, :].copy()

            Greenline_ang = cv2.erode(
                Greenline_ang,
                KERNEL_3,
                iterations=2
            )

            Greenline_ang = cv2.dilate(
                Greenline_ang,
                KERNEL_3,
                iterations=4
            )

            contours_ang, _ = cv2.findContours(
                Greenline_ang.copy(),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )


            # state van dua theo line o gan robot
            state = len(contours_err)

            # =================================================
            # ROS MESSAGE
            # =================================================

            msg = LineData()

            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.state = state

            # =================================================
            # CO LINE
            # =================================================

            if state > 0:
                # =================================================
            # CONTOUR CHO ERROR
            # =================================================

                main_contour_err = max(
                    contours_err,
                    key=cv2.contourArea
                )

                blackbox_err = cv2.minAreaRect(main_contour_err)

                (x_min, y_min), (w_err, h_err), _ = blackbox_err


                # =================================================
                # CONTOUR CHO ANG
                # =================================================

                # =================================================
                # CONTOUR CHO ANG
                # =================================================

                if len(contours_ang) > 0:

                    main_contour_ang = max(
                        contours_ang,
                        key=cv2.contourArea
                    )

                    blackbox_ang = cv2.minAreaRect(main_contour_ang)

                    (x_ang, y_ang), (w_min, h_min), ang = blackbox_ang

                    if ang < -45:
                        ang = 90 + ang

                    if w_min < h_min and ang > 0:
                        ang = (90 - ang) * -1

                    if w_min > h_min and ang < 0:
                        ang = 90 + ang

                    ang_raw = ang

                else:
                    # Khong co line trong ROI tinh ANG
                    # giu lai goc gan nhat
                    ang_raw = self.ang_send_last

                self.ang_median_buffer.append(ang_raw)

                ang_median = float(np.median(self.ang_median_buffer))

                if not self.ang_filter_initialized:
                    self.ang_filtered = ang_median
                    self.ang_filter_initialized = True
                else:
                    self.ang_filtered = ANG_ALPHA * ang_median + (1.0 - ANG_ALPHA) * self.ang_filtered

                if not self.ang_send_initialized:
                    self.ang_send_last = self.ang_filtered
                    self.ang_send_initialized = True
                else:
                    if abs(self.ang_filtered - self.ang_send_last) >= ANG_DEADBAND:
                        self.ang_send_last = self.ang_filtered

                ang_send = int(round(self.ang_send_last))

                # =============================================
                # ERROR
                # =============================================

                setpoint = width // 2

                error_raw = int(setpoint - x_min) * K_SCALE_X

                if not self.error_filter_initialized:
                    self.error_filtered = error_raw
                    self.error_filter_initialized = True
                else:
                    self.error_filtered = ERROR_ALPHA * error_raw + (1.0 - ERROR_ALPHA) * self.error_filtered

                error_send = round(self.error_filtered, 2)

                # =============================================
                # CURVE SEND
                # =============================================

                if curve_valid and self.curve_filter_initialized:
                    curve_send = round(self.curve_filtered, 2)
                else:
                    curve_send = 0.0

                # =============================================
                # DATA
                # =============================================

                msg.angle = float(ang_send)
                msg.error = float(error_send)
                msg.curve = float(curve_send)

                # =============================================
                # DISPLAY
                # =============================================

                if self.show_image:
                    cv2.putText(bottom_half, "Ang: %d" % ang_send, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1)
                    cv2.putText(bottom_half, "Err: %.2f" % error_send, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1)
                    cv2.putText(bottom_half, "Crv: %.2f" % curve_send, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
                    cv2.putText(imgWarp, "FPS: %.1f" % self.fps_filtered, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    cv2.line(bottom_half, (int(x_min), 50), (int(x_min), 100), (0, 0, 255), 2)

            # =================================================
            # MAT LINE
            # =================================================

            else:
                msg.angle = 0.0
                msg.error = 0.0
                msg.curve = 0.0

                self.reset_filters()

            # =================================================
            # PUBLISH
            # =================================================

            self.data_pub.publish(msg)

            # =================================================
            # TRUC GIUA
            # =================================================

            if self.publish_debug_image or self.show_image:
                mid_x = width // 2
                mid_y = height // 2

                cv2.line(imgWarp, (mid_x, height - 1), (mid_x, mid_y), (0, 255, 255), 1)

            # =================================================
            # ROS DEBUG IMAGE
            # =================================================

            if self.publish_debug_image and self.image_pub.get_num_connections() > 0:
                img_msg = cv2_to_imgmsg(imgWarp, encoding="bgr8")

                img_msg.header.stamp = msg.header.stamp
                img_msg.header.frame_id = self.frame_id

                self.image_pub.publish(img_msg)

            # =================================================
            # SHOW
            # =================================================

            if self.show_image:
                cv2.imshow("final image", imgWarp)
                #cv2.imshow("HSV Mask", full_line)
                cv2.waitKey(1)

            #self.rate.sleep()

        cv2.destroyAllWindows()


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    try:
        node = KinectLineNode()
        node.spin()

    except rospy.ROSInterruptException:
        pass