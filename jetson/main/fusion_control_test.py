#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import time
import rospy
import serial
from sensor_msgs.msg import LaserScan
from kinect_line_detector.msg import LineData

MODE_LINE_FOLLOWING = 0
MODE_OBSTACLE_AVOIDANCE = 1
MODE_SAFE_STOP = 2

ANG_STRAIGHT = "straight"
ANG_LEFT = "left"
ANG_RIGHT = "right"


def normalize_deg(angle_deg):
    """Dua goc (do) ve khoang (-180, 180]."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def sign(x):
    """Tra ve 1.0 neu x >= 0, -1.0 neu x < 0."""
    return 1.0 if x >= 0 else -1.0


class FusionControlNode(object):
    def __init__(self):
        rospy.init_node("fusion_control_node")

        self._shutting_down = False
        # --- Tham so Serial ---
        self.port = rospy.get_param("~port", "/dev/arduino")
        self.baudrate = rospy.get_param("~baudrate", 115200)

        #self.post_avoid_hold_frames = rospy.get_param("~post_avoid_hold_frames", 2)  # so frame dung tam sau khi thoat avoidance
        #self.post_avoid_hold_count = 0

        # --- Tham so vung / nguong khoang cach (dung cho chuyen mode) ---
        self.robot_width = rospy.get_param("~robot_width", 0.40)
        self.margin_a2 = rospy.get_param("~margin_a2", 0.05)
        self.enter_dist = rospy.get_param("~enter_dist", 0.65)
        self.exit_dist = rospy.get_param("~exit_dist", 0.70)
        self.half_corridor = self.robot_width / 2.0 + self.margin_a2

        # --- Tham so goc (vung dem hysteresis, dung cho chuyen mode) ---
        self.straight_ang_th = rospy.get_param("~straight_ang_th", 10.0)
        self.ang_hysteresis = rospy.get_param("~ang_hysteresis", 5.0)

        # --- Tham so exit theo line ---
        self.err_exit_th = rospy.get_param("~err_exit_th", 0.20)

        # --- Debounce chuyen mode ---
        self.debounce_frames = rospy.get_param("~debounce_frames", 2)
        self.enter_count = 0
        self.exit_count = 0

        # --- Timeout an toan ---
        self.data_timeout = rospy.get_param("~data_timeout", 0.5)

        # --- Tham so thuat toan PCA + duong ao ---
        self.g_obs = rospy.get_param("~g_obs", 0.65)
        self.d_off = rospy.get_param("~d_off", 0.45)
        self.side_lock_frames = rospy.get_param("~side_lock_frames", 3)
        self.continuity_jump_th = rospy.get_param("~continuity_jump_th", 90.0)
        self.ang_max_virtual = rospy.get_param("~ang_max_virtual", 45.0)

        self.avoid_side_state = None
        self.side_buffer = []
        self.theta_T_prev_deg = None

        # --- State machine ---
        self.mode = MODE_LINE_FOLLOWING
        self.ang_state = ANG_STRAIGHT

        # --- Du lieu moi nhat ---
        self.last_line = None
        self.last_scan = None

        # --- Ket noi Serial ---
        self.ser = None

        self.enable_avoidance = rospy.get_param("~enable_avoidance", True)

        # --- Bien do thoi gian giua 2 lan goi control_loop ---
        self._last_loop_time = None
        self.seq_counter = 0

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.01, write_timeout=0.05)
            rospy.loginfo("Da mo cong Serial, cho Arduino reset...")
            rospy.sleep(2.0)
            rospy.loginfo("Da ket noi Serial tai %s @ %d", self.port, self.baudrate)
        except serial.SerialException as e:
            rospy.logerr("Khong the mo Serial %s: %s", self.port, str(e))

        # --- Subscriber ---
        rospy.Subscriber("/kinect/line_data", LineData, self.line_callback, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)

        # --- Timer dieu phoi ---
        control_rate = rospy.get_param("~control_rate", 20.0)
        rospy.Timer(rospy.Duration(1.0 / control_rate), self.control_loop)

        rospy.loginfo("fusion_control_node da san sang.")

    # ------------------------------------------------------------------
    def line_callback(self, msg):
        self.last_line = msg

    def scan_callback(self, msg):
        self.last_scan = msg

    # ------------------------------------------------------------------
    def classify_zones(self, scan_msg):
        min_A = min_B = min_C = float('inf')
        for i, r in enumerate(scan_msg.ranges):
            if not math.isfinite(r):
                continue
            if r < scan_msg.range_min or r > scan_msg.range_max:
                continue
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            if angle < -math.pi / 2 or angle > math.pi / 2:
                continue

            u = r * math.cos(angle)
            w = r * math.sin(angle)
            if u <= 0:
                continue

            if -self.half_corridor <= w <= self.half_corridor:
                min_A = min(min_A, r)
            elif w > self.half_corridor:
                min_B = min(min_B, r)
            else:
                min_C = min(min_C, r)
        return min_A, min_B, min_C

    # ------------------------------------------------------------------
    def update_ang_state(self, ang, line_state):
            if line_state <= 0:
                # Mat line: du lieu ang khong dang tin, giu nguyen trang thai truoc do
                return self.ang_state

            th = self.straight_ang_th
            hys = self.ang_hysteresis

            if self.ang_state == ANG_STRAIGHT:
                if ang > th + hys:
                    self.ang_state = ANG_RIGHT
                elif ang < -(th + hys):
                    self.ang_state = ANG_LEFT
            elif self.ang_state == ANG_RIGHT:
                if ang < th - hys:
                    self.ang_state = ANG_STRAIGHT
            elif self.ang_state == ANG_LEFT:
                if ang > -(th - hys):
                    self.ang_state = ANG_STRAIGHT

            return self.ang_state

    # ------------------------------------------------------------------
    def should_enter_avoidance(self, min_A, min_B, min_C, ang_state):
        has_A = min_A < self.enter_dist
        has_B = min_B < self.enter_dist
        has_C = min_C < self.enter_dist

        if has_A and ang_state == ANG_STRAIGHT:
            return True
        if (has_A or has_B) and ang_state == ANG_LEFT:
            return True
        if (has_A or has_C) and ang_state == ANG_RIGHT:
            return True
        return False

    def should_exit_avoidance(self, min_A, min_B, min_C, line_msg):
        if line_msg is None or line_msg.state <= 0:
            return False
        if abs(line_msg.error) >= self.err_exit_th:
            return False
        if min_A <= self.exit_dist or min_B <= self.exit_dist or min_C <= self.exit_dist:
            return False
        return True

    # ------------------------------------------------------------------
    def compute_avoidance_cmd(self):
        """Sinh (angle_ao_deg, error_ao_m) tu du lieu Lidar theo thuat toan
        PCA + duong ao. Tra ve None neu khong co dai vat can trong tam quet
        (khong doi trang thai side-lock trong truong hop nay).
        """
        scan = self.last_scan
        if scan is None:
            return None

        # --- Buoc 1: doc du lieu tho + chuyen sang Descartes khung than ---
        alphas, gs = [], []
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            alphas.append(angle)
            gs.append(r)

        if not gs:
            return None

        # --- Buoc 1b: phat hien dai vat can lien tiep g_i < g_obs ---
        band_idx = [k for k, g in enumerate(gs) if g < self.g_obs]
        if not band_idx:
            # Khong co vat can trong tam quet -> khong sinh duong ao
            return None

        segments, seg_start, prev = [], band_idx[0], band_idx[0]
        for k in band_idx[1:]:
            if k == prev + 1:
                prev = k
            else:
                segments.append((seg_start, prev))
                seg_start, prev = k, k
        segments.append((seg_start, prev))
        iL, iR = max(segments, key=lambda s: s[1] - s[0])

        alpha_iL, alpha_iR = alphas[iL], alphas[iR]

        # --- Buoc 2: O = diem gan robot nhat trong toan bo tam quet ---
        idx_O = min(range(len(gs)), key=lambda k: gs[k])
        g_O = gs[idx_O]
        alpha_O = alphas[idx_O]
        u_O = g_O * math.cos(alpha_O)
        w_O = g_O * math.sin(alpha_O)

        # --- Buoc 3: T_hat = huong bien bang PCA tren toan dai {iL..iR} ---
        cluster = [(gs[k] * math.cos(alphas[k]), gs[k] * math.sin(alphas[k]))
                   for k in range(iL, iR + 1)]
        n_pts = len(cluster)
        ubar = sum(p[0] for p in cluster) / n_pts
        wbar = sum(p[1] for p in cluster) / n_pts
        Cuu = sum((p[0] - ubar) ** 2 for p in cluster) / n_pts
        Cww = sum((p[1] - wbar) ** 2 for p in cluster) / n_pts
        Cuw = sum((p[0] - ubar) * (p[1] - wbar) for p in cluster) / n_pts

        theta_T_rad = 0.5 * math.atan2(2.0 * Cuw, Cuu - Cww)
        theta_T_deg = math.degrees(theta_T_rad)

        # --- Buoc 4: kiem tra lien tuc goc voi frame truoc, gap 180 deg neu nhay >90 deg ---
        if self.theta_T_prev_deg is not None:
            diff = normalize_deg(theta_T_deg - self.theta_T_prev_deg)
            if abs(diff) > self.continuity_jump_th:
                theta_T_deg = theta_T_deg - 180.0 * sign(theta_T_deg)
        self.theta_T_prev_deg = theta_T_deg
        theta_T_rad = math.radians(theta_T_deg)
        Tu, Tw = math.cos(theta_T_rad), math.sin(theta_T_rad)

        # --- Buoc 5 + 5.5: chon N_hat (tro ra vung trong) + chot chieu tranh ---
        if self.avoid_side_state is None:
            self.side_buffer.append((alpha_iL, alpha_iR))
            if len(self.side_buffer) < self.side_lock_frames:
                # Chua du frame de chot ben -> chua sinh duong ao, giu nguyen
                return None
            avg_aL = sum(a for a, _ in self.side_buffer) / len(self.side_buffer)
            avg_aR = sum(b for _, b in self.side_buffer) / len(self.side_buffer)
            self.avoid_side_state = 'L' if abs(avg_aL) < abs(avg_aR) else 'R'
            self.side_buffer = []
            rospy.loginfo("Da chot chieu tranh: %s (avg_aL=%.1f deg, avg_aR=%.1f deg)",
                           self.avoid_side_state, math.degrees(avg_aL), math.degrees(avg_aR))

        side = self.avoid_side_state
        i_edge = iL if side == 'L' else iR
        u_E = gs[i_edge] * math.cos(alphas[i_edge])
        w_E = gs[i_edge] * math.sin(alphas[i_edge])

        # Hai nhanh phap tuyen cua T, chon nhanh dong huong voi E (ben da chot)
        N1u, N1w = Tw, -Tu
        N2u, N2w = -Tw, Tu
        dot1 = N1u * u_E + N1w * w_E
        dot2 = N2u * u_E + N2w * w_E
        Nu, Nw = (N1u, N1w) if dot1 < dot2 else (N2u, N2w)

        # --- Buoc 6 + 7: d_off co dinh + Q neo tai O ---
        # error_ao = toa do w cua Q (Q = O + d_off*N_hat) = khoang cach tu Q den truc u
        d_off = self.d_off
        error_ao = w_O + d_off * Nw

        # --- Buoc 9: angle_ao = theta_T (huong bien) ---
        angle_ao_deg = theta_T_deg

        # --- Buoc 11: gioi han vat ly ---
        angle_ao_deg = max(-self.ang_max_virtual, min(self.ang_max_virtual, angle_ao_deg))

        return angle_ao_deg, error_ao

    # ------------------------------------------------------------------
    def check_data_fresh(self):
        now = rospy.Time.now()
        if self.last_line is None or (now - self.last_line.header.stamp).to_sec() > self.data_timeout:
            return False
        if self.enable_avoidance:
            if self.last_scan is None or (now - self.last_scan.header.stamp).to_sec() > self.data_timeout:
                return False
        return True

    # ------------------------------------------------------------------
    def control_loop(self, event):
        # ---- DO THOI GIAN GIUA 2 LAN GOI control_loop (CHAN DOAN) ----
        t_enter = time.time()
        if self._last_loop_time is not None:
            gap = t_enter - self._last_loop_time
            if gap > 0.1:  # canh bao neu lon hon 5 lan chu ky binh thuong (0.05s)
                rospy.logwarn("CANH BAO: khoang cach giua 2 lan goi control_loop = %.3f s", gap)
        self._last_loop_time = t_enter

        if not self.check_data_fresh():
            rospy.logwarn_throttle(1.0, "Mat du lieu cam bien (qua %.2fs), DUNG XE", self.data_timeout)
            self.send_cmd(0, 0.0, 0, MODE_SAFE_STOP, curve=0.0)
            self.enter_count = 0
            self.exit_count = 0
            self._log_loop_duration(t_enter, "check_data_fresh_fail")
            return

        if self.enable_avoidance:
            min_A, min_B, min_C = self.classify_zones(self.last_scan)
        else:
            min_A = min_B = min_C = float('inf')

        ang_state = self.update_ang_state(self.last_line.angle, self.last_line.state)

        if self.mode == MODE_LINE_FOLLOWING:
            if self.enable_avoidance and self.should_enter_avoidance(min_A, min_B, min_C, ang_state):
                self.enter_count += 1
            else:
                self.enter_count = 0

            if self.enter_count >= self.debounce_frames:
                self.mode = MODE_OBSTACLE_AVOIDANCE
                self.enter_count = 0
                self.exit_count = 0
                self.avoid_side_state = None
                self.side_buffer = []
                self.theta_T_prev_deg = None
                rospy.logwarn("Chuyen mode: LINE_FOLLOWING -> OBSTACLE_AVOIDANCE "
                               "(A=%.2f B=%.2f C=%.2f ang_state=%s)",
                               min_A, min_B, min_C, ang_state)

        elif self.mode == MODE_OBSTACLE_AVOIDANCE:
            if self.should_exit_avoidance(min_A, min_B, min_C, self.last_line):
                self.exit_count += 1
            else:
                self.exit_count = 0

            if self.exit_count >= self.debounce_frames:
                self.mode = MODE_LINE_FOLLOWING
                self.enter_count = 0
                self.exit_count = 0
                #self.post_avoid_hold_count = self.post_avoid_hold_frames   # BAT DAU DEM NGUOC
                rospy.logwarn("Chuyen mode: OBSTACLE_AVOIDANCE -> LINE_FOLLOWING")

        '''if self.mode == MODE_LINE_FOLLOWING:
            if self.post_avoid_hold_count > 0:
                self.post_avoid_hold_count -= 1
                rospy.logwarn_throttle(1.0, "Vua thoat avoidance, DUNG XE tam thoi (%d frame con lai)",
                                        self.post_avoid_hold_count)
                self.send_cmd(0, 0.0, 0, MODE_LINE_FOLLOWING, curve=0.0)
            elif self.last_line.state <= 0:
                rospy.logwarn_throttle(1.0, "Mat line, gui state=0 de DUNG XE")
                self.send_cmd(0, 0.0, 0, MODE_LINE_FOLLOWING, curve=0.0)
            else:
                self.send_cmd(self.last_line.angle, self.last_line.error,
                               self.last_line.state, MODE_LINE_FOLLOWING,
                               curve=self.last_line.curve)'''
        if self.mode == MODE_LINE_FOLLOWING:
            if self.last_line.state <= 0:
                rospy.logwarn_throttle(1.0, "Mat line, gui state=0 de DUNG XE")
                self.send_cmd(0, 0.0, 0, MODE_LINE_FOLLOWING, curve=0.0)
            else:
                self.send_cmd(
                    self.last_line.angle,
                    self.last_line.error,
                    self.last_line.state,
                    MODE_LINE_FOLLOWING,
                    curve=self.last_line.curve
                )
        else:
            result = self.compute_avoidance_cmd()
            if result is not None:
                angle_ao, error_ao = result
                self.send_cmd(-angle_ao, error_ao, 1, MODE_OBSTACLE_AVOIDANCE, curve=0.0)
            else:
                '''rospy.logwarn_throttle(1.0, "Khong tinh duoc duong ao, DUNG XE tam thoi")
                self.send_cmd(0, 0.0, 0, MODE_OBSTACLE_AVOIDANCE, curve=0.0)'''
                rospy.logwarn_throttle(
                    1.0,
                    "Chua tinh duoc duong ao, tam thoi tiep tuc bam line"
                )

                if self.last_line is not None and self.last_line.state > 0:
                    self.send_cmd(
                        self.last_line.angle,
                        self.last_line.error,
                        self.last_line.state,
                        MODE_LINE_FOLLOWING,
                        curve=self.last_line.curve
                    )
                else:
                    self.send_cmd(0, 0.0, 0, MODE_SAFE_STOP, curve=0.0)

        self._log_loop_duration(t_enter, "normal")

    def _log_loop_duration(self, t_enter, tag):
        """Do va in thoi gian XU LY BEN TRONG 1 lan goi control_loop (khong phai
        khoang cach giua 2 lan goi, ma la thoi gian ham nay chay bao lau)."""
        duration = time.time() - t_enter
        if duration > 0.05:  # canh bao neu 1 lan xu ly ton hon 1 chu ky (0.05s)
            rospy.logwarn("CANH BAO: control_loop (%s) chay mat %.3f s", tag, duration)

    # ------------------------------------------------------------------
    def send_cmd(self, ang, error, state, mode, curve=0.0):
        """Format chung cho moi mode: ang#error#state#mode#curve#seq"""
        if self._shutting_down or self.ser is None or not self.ser.is_open:
            return

        self.seq_counter += 1
        try:
            msg = "%d#%.4f#%d#%d#%.4f#%d\n" % (
                int(ang), error, int(state), int(mode), curve, self.seq_counter)
            self.ser.write(msg.encode())

            max_reads = 20
            count = 0
            while self.ser.in_waiting > 0 and count < max_reads:
                debug_line = self.ser.readline().decode(errors="ignore").strip()
                if debug_line:
                    rospy.loginfo(debug_line)
                count += 1
        except serial.SerialTimeoutException:
            rospy.logwarn("Serial write timeout")
        except serial.SerialException as e:
            rospy.logerr("Loi Serial: %s", str(e))

    # ------------------------------------------------------------------
    def shutdown(self):
        # Gui lenh dung khan cap TRUC TIEP, khong qua send_cmd (vi se bi chan boi _shutting_down)
        if self.ser is not None and self.ser.is_open:
            try:
                msg = "0#0.0000#0#%d#0.0000#0\n" % MODE_SAFE_STOP
                self.ser.write(msg.encode())
            except (serial.SerialException, serial.SerialTimeoutException):
                pass

        self._shutting_down = True
        time.sleep(0.1)  # cho control_loop dang chay (neu co) kip thoat, khong goi send_cmd nua

        if self.ser is not None and self.ser.is_open:
            self.ser.close()


if __name__ == "__main__":
    node = FusionControlNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()