#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
IMU姿态融合最终示例代码（进一步增强版：Yaw轴ZUPT + “忽略Yaw重力预测” 约束）
-----------------------------------------------------------------------
在原先6轴卡尔曼 + 互补滤波 + ZUPT基础上，进一步改进Yaw约束方式：
  - 不再简单依赖 X/Y 加速度差值 delta_a_xy，
  - 改为：将当前预测姿态的 (roll, pitch) 与 yaw=0 的假设相结合，计算出
    “若无Yaw旋转，传感器应当测到的重力矢量” 与 实际加速度之间的残差 residual_xy。
  - residual_xy 越小，说明纯Roll/Pitch即可解释当前加速度分布，则应对Yaw施加更强约束；
    residual_xy 越大，说明存在实际绕Z旋转或横向加速度，不应过度锁定Yaw。

默认输入：
  - 加速度 (ax, ay, az): 单位 m/s^2
  - 陀螺仪 (gx, gy, gz): 单位 rad/s
  - 采样间隔 dt_us：单位微秒
  - 温度 temp_c：摄氏度，接口保留，目前未实现补偿

主要流程：
  1) 使用 IMUFilter 对6轴原始数据做卡尔曼滤波，得到滤波后 ax_f, ay_f, az_f, gx_f, gy_f, gz_f；
  2) 在 IMUAHRS 中先进行“忽略Yaw的重力预测”，并与实际加速度对比得到 residual_xy；
  3) 将 residual_xy 转换为权重 w_yaw；再结合 ZUPT 补偿后的 gz_comp 进行融合；
  4) 只对 yaw 做此额外约束，roll/pitch 仍沿用互补滤波与加速度融合。

如需扩展磁力计或温度补偿，可在此基础上自行实现。
"""

import math
import numpy as np

# =============================================================================
# 一、Kalman1D：一维卡尔曼滤波器，用于加速度计每个轴数据
# =============================================================================
class Kalman1D:
    """
    一维卡尔曼滤波器，适用于加速度计单轴数据。
    状态转移简化：x直接保持不变；p_pred = p + q。
    测量：z = x + 噪声，测量噪声为 r。
    """
    def __init__(self, init_x=0.0, init_p=1.0, q=0.001, r=0.1):
        """
        参数:
          init_x : 初始状态
          init_p : 初始状态协方差
          q      : 过程噪声
          r      : 测量噪声
        """
        self.x = init_x  # 状态量
        self.p = init_p  # 协方差
        self.q = q       # 过程噪声
        self.r = r       # 测量噪声

    def update(self, z):
        """
        单步卡尔曼更新:
          z: 当前测量值
        返回:
          滤波后的状态
        """
        # 预测
        x_pred = self.x
        p_pred = self.p + self.q

        # 卡尔曼增益
        K = p_pred / (p_pred + self.r)

        # 更新
        self.x = x_pred + K * (z - x_pred)
        self.p = (1.0 - K) * p_pred
        return self.x


# =============================================================================
# 二、Kalman2D：二维卡尔曼滤波器，用于单轴陀螺仪 [angle, rate]
# =============================================================================
class Kalman2D:
    """
    二维卡尔曼滤波器，状态向量为[angle, rate]，分别表示角度(积分得到)与角速度。
    观测仅有角速度 rate_meas。
    根据采样周期 dt_s 动态缩放过程噪声。
    """
    def __init__(self,
                 angle_init=0.0,
                 rate_init=0.0,
                 base_q=1e-5,
                 p_init=None,
                 r=None):
        """
        参数:
          angle_init: 初始角度，单位 rad
          rate_init : 初始角速度，单位 rad/s
          base_q    : 基准过程噪声，用于和dt_s一起计算真实Q
          p_init    : 初始协方差矩阵(2x2)
          r         : 测量噪声(标量)，默认为1e-3
        """
        # 状态向量 x = [angle, rate]
        self.x = np.array([angle_init, rate_init], dtype=float)

        # 状态协方差矩阵 P
        if p_init is None:
            sigma_angle = 0.087  # 假设角度标准差约0.087 rad (5度)
            sigma_rate = 0.01  # 假设角速度标准差约0.01 rad/s
            rho = 0.8  # 角度和角速度相关系数
            self.P = np.array([
                [sigma_angle ** 2, rho * sigma_angle * sigma_rate],
                [rho * sigma_angle * sigma_rate, sigma_rate ** 2]
            ], dtype=float)
        else:
            self.P = p_init.astype(float)

        # 过程噪声的基准值
        self.base_q = base_q

        # 测量噪声(标量)，若不指定则取1e-3
        self.R = 1e-3 if (r is None) else float(r)

        # 观测矩阵 H: 仅观测 rate
        self.H = np.array([[0.0, 1.0]], dtype=float)
        self.I = np.eye(2, dtype=float)

    def update(self, rate_meas, dt_s):
        """
        单步更新:
          rate_meas: 当前时刻的角速度测量(rad/s)
          dt_s     : 采样周期(秒)
        返回 (angle, rate): 滤波后的角度与角速度
        """
        # 状态转移矩阵 F
        F = np.array([
            [1.0, dt_s],
            [0.0, 1.0]
        ], dtype=float)

        # 过程噪声 Q，随 dt_s 变化
        Q_scaled = np.array([
            [0.25 * dt_s**4, 0.5 * dt_s**3],
            [0.5 * dt_s**3,  dt_s**2]
        ], dtype=float) * self.base_q

        # 预测
        x_pred = F @ self.x
        P_pred = F @ self.P @ F.T + Q_scaled

        # 观测 z
        z = np.array([rate_meas], dtype=float)

        # 卡尔曼增益
        S = self.H @ P_pred @ self.H.T + self.R
        S_val = max(S[0, 0], 1e-12)  # 防止除0
        K = (P_pred @ self.H.T) / S_val

        # 更新
        y = z - (self.H @ x_pred)
        x_new = x_pred + K @ y
        P_new = (self.I - K @ self.H) @ P_pred

        self.x = x_new
        self.P = P_new

        # 返回当前滤波后的 (angle, rate)
        return float(self.x[0]), float(self.x[1])


# =============================================================================
# 三、YawDriftCompensator：用于Z轴零速率检测与偏置估计（ZUPT）
# =============================================================================
class YawDriftCompensator:
    """
    通过ZUPT思路，在检测到设备静止或低动态时估计Yaw轴偏置(yaw_bias)，
    并在后续更新中对陀螺仪Z值进行补偿，减少长时间积分导致的漂移。
    """
    def __init__(self, beta=0.99):
        """
        参数:
          beta: 指数平滑因子，越接近1表示偏置更新越缓慢。
        """
        self.beta = beta          # 指数平滑系数
        self.yaw_bias = 0.0       # Z轴的偏置初值，可根据经验或标定初始化

    def update_bias(self, gyro_z, dt, is_static):
        """
        根据是否静止状态，更新yaw偏置并返回补偿后的陀螺仪z轴值。
        参数:
          gyro_z   : 当前滤波后z轴角速度(rad/s)
          dt       : 采样周期(秒)
          is_static: 是否检测到静止(或低动态)
        返回:
          gz_comp  : 补偿后的z轴角速度
        """
        if is_static:
            # 若判断为静止或低动态，则更新偏置
            self.yaw_bias = self.beta * self.yaw_bias + (1 - self.beta) * gyro_z

        # 返回补偿后的角速度
        gz_comp = gyro_z - self.yaw_bias
        return gz_comp

    @staticmethod
    def main_test():
        """
        简单单元测试，用以演示YawDriftCompensator的典型调用方式。
        模拟若干帧陀螺仪z值，然后在静止与非静止间切换。
        """
        comp = YawDriftCompensator(beta=0.95)
        test_data = [
            # (gyro_z, dt, is_static)
            (0.01, 0.01, True),
            (0.02, 0.01, True),
            (0.00, 0.01, True),
            (0.10, 0.01, False),
            (0.15, 0.01, False),
            (0.00, 0.01, True),
        ]
        for i, (gz, dt, st) in enumerate(test_data):
            gz_comp = comp.update_bias(gz, dt, st)
            print(f"[Frame {i}] raw gz={gz:.3f}, is_static={st}, bias={comp.yaw_bias:.4f}, gz_comp={gz_comp:.3f}")


# =============================================================================
# 四、IMUFilter：对IMU (ax, ay, az, gx, gy, gz) 的卡尔曼滤波预处理
# =============================================================================
class IMUFilter:
    """
    组合三轴 Kalman1D (加速度) 与三轴 Kalman2D (陀螺仪)，
    并根据简单的运动状态检测（acc_norm 与 gyro_norm）自适应调整陀螺仪的Q。
    """
    def __init__(self):
        # 三轴加速度滤波器，默认Z轴在静止时约等于9.81
        self.accelX = Kalman1D(init_x=0.0,  init_p=1.0, q=0.001, r=0.1)
        self.accelY = Kalman1D(init_x=0.0,  init_p=1.0, q=0.001, r=0.1)
        self.accelZ = Kalman1D(init_x=9.81, init_p=1.0, q=0.001, r=0.1)

        # 三轴陀螺仪2D卡尔曼滤波器
        self.gyroX2D = Kalman2D(angle_init=0.0, rate_init=0.0, base_q=1e-3, r=1e-1)
        self.gyroY2D = Kalman2D(angle_init=0.0, rate_init=0.0, base_q=1e-3, r=1e-1)
        self.gyroZ2D = Kalman2D(angle_init=0.0, rate_init=0.0, base_q=0.5,  r=1e-4)

        # 存储每个轴的“卡尔曼估计角度”，主要用于调试或参考
        self.gyroAngleX = 0.0
        self.gyroAngleY = 0.0
        self.gyroAngleZ = 0.0

    def _detect_motion_state(self, acc_norm, gyro_norm):
        """
        基于加速度与角速度范数判断运动状态:
          - static       : 静止
          - low_dynamic  : 低动态
          - high_dynamic : 高动态
        """
        if gyro_norm < 0.1 and abs(acc_norm - 9.81) < 0.2:
            return 'static'
        elif gyro_norm < 0.5:
            return 'low_dynamic'
        else:
            return 'high_dynamic'

    def _adjust_kalman_parameters(self, motion_state):
        """
        根据运动状态动态调整 Kalman2D 的 base_q, 使滤波器对高动态场景更敏感。
        这里只对X和Y做了自适应；Z轴的 base_q 不在这里改动，可视情需求。
        """
        if motion_state == 'static':
            new_q = 1e-4
        elif motion_state == 'low_dynamic':
            new_q = 1e-3
        else:
            new_q = 1e-2
        self.gyroX2D.base_q = new_q
        self.gyroY2D.base_q = new_q
        # self.gyroZ2D.base_q = new_q  # 若需要也可一起改

    def update_all(self, ax_raw, ay_raw, az_raw,
                   gx_raw, gy_raw, gz_raw, dt_s):
        """
        对IMU六轴数据进行卡尔曼滤波:
          ax_raw, ay_raw, az_raw: 加速度 m/s^2
          gx_raw, gy_raw, gz_raw: 角速度 rad/s
          dt_s: 采样间隔(秒)
        返回: (ax_f, ay_f, az_f, gx_f, gy_f, gz_f) 滤波后的数据
        """
        # 1) 判断运动状态并调整陀螺仪滤波器参数
        acc_norm = math.sqrt(ax_raw*ax_raw + ay_raw*ay_raw + az_raw*az_raw)
        gyro_norm = math.sqrt(gx_raw*gx_raw + gy_raw*gy_raw + gz_raw*gz_raw)
        motion_state = self._detect_motion_state(acc_norm, gyro_norm)
        self._adjust_kalman_parameters(motion_state)

        # 2) 加速度计三轴滤波
        ax_f = self.accelX.update(ax_raw)
        ay_f = self.accelY.update(ay_raw)
        az_f = self.accelZ.update(az_raw)

        # 3) 陀螺仪三轴滤波(2D Kalman)，得到滤波后的角速度
        self.gyroAngleX, gx_f_out = self.gyroX2D.update(gx_raw, dt_s)
        self.gyroAngleY, gy_f_out = self.gyroY2D.update(gy_raw, dt_s)
        self.gyroAngleZ, gz_f_out = self.gyroZ2D.update(gz_raw, dt_s)

        return ax_f, ay_f, az_f, gx_f_out, gy_f_out, gz_f_out


# =============================================================================
# 五、IMUAHRS：姿态融合（卡尔曼 + 互补滤波 + 四元数） + [ZUPT + “忽略Yaw重力预测”] 附加增强
# =============================================================================
class IMUAHRS:
    """
    主要流程:
      1. 用 IMUFilter 对6轴原始数据进行卡尔曼滤波
      2. 使用滤波后的陀螺仪角速度做四元数积分，得到预测姿态
      3. 从加速度推算 roll/pitch，通过互补滤波修正预测姿态
      4. 引入 Z 轴漂移校正(零速率补偿)，并利用“忽略Yaw后重力预测”来区分Roll/Pitch vs.真实Yaw
      5. 返回融合后的姿态（四元数 或 欧拉角）
    """
    def __init__(self, default_freq=100.0):
        # 初始化姿态为单位四元数
        self.q_w = 1.0
        self.q_x = 0.0
        self.q_y = 0.0
        self.q_z = 0.0

        self.initialized = False
        self.defaultFrequency = default_freq
        self.imuFilter = IMUFilter()

        # 互补滤波中需记录上次加速度范数、上次dt_s
        self._last_acc_norm = None
        self._last_dt_s = None

        # 创建 yaw 漂移补偿器
        self.yawDriftComp = YawDriftCompensator(beta=0.99)

    def update(self, ax_raw, ay_raw, az_raw,
               gx_raw, gy_raw, gz_raw,
               dt_us, temp_c=25.0):
        """
        主更新函数:
          ax_raw, ay_raw, az_raw: 加速度(m/s^2)
          gx_raw, gy_raw, gz_raw: 角速度(rad/s)
          dt_us: 采样间隔(微秒)
          temp_c: 当前温度(摄氏度)，预留给温度补偿(暂无实现)
        返回:
          (qw, qx, qy, qz) 最新姿态四元数
        """
        # 1) 基础输入检查
        if dt_us <= 0:
            raise ValueError("dt_us must be positive")
        for val in [ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw]:
            if not isinstance(val, (int, float)):
                raise TypeError("All sensor inputs must be numeric")

        # 2) 若尚未初始化，则通过加速度设置初始姿态(Roll/Pitch, Yaw=0)
        if not self.initialized:
            self._initialize_by_accel(ax_raw, ay_raw, az_raw)

        dt_s = dt_us * 1e-6
        self._last_dt_s = dt_s

        # 3) 调用 IMUFilter，获取滤波后的 (ax_f, ay_f, az_f, gx_f, gy_f, gz_f)
        ax_f, ay_f, az_f, gx_f, gy_f, gz_f = self.imuFilter.update_all(
            ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, dt_s
        )

        # 4) 根据当前预测姿态(roll, pitch) [不考虑yaw] 计算“理论重力g_pred”，进而得到 XY 残差
        #    首先，先根据上次姿态做一次四元数积分，得到“预测姿态 pred_xxx”
        dq_w, dq_x, dq_y, dq_z = self._gyro_to_quat_delta(gx_f, gy_f, gz_f, dt_s)
        pred_w, pred_x, pred_y, pred_z = self._quat_multiply(
            self.q_w, self.q_x, self.q_y, self.q_z,
            dq_w, dq_x, dq_y, dq_z
        )
        pred_roll, pred_pitch, pred_yaw = self._quat_to_euler(pred_w, pred_x, pred_y, pred_z)

        # 计算: 若 yaw=0, roll=pred_roll, pitch=pred_pitch 时，设备坐标下的重力向量
        gpx, gpy, gpz = self._compute_gravity_ignore_yaw(pred_roll, pred_pitch)
        # 与实际测得的 (ax_f, ay_f) 比较，得到残差 residual_xy
        residual_xy = math.sqrt((ax_f - gpx)**2 + (ay_f - gpy)**2)

        # 设置一个阈值 T，用于构造权重 w_yaw
        # residual_xy 越小 -> 无需 yaw 大幅变化 -> 权重越大
        T = 0.5
        w_yaw = max(0.0, 1.0 - (residual_xy / T))

        # 5) ZUPT 补偿: 判断是否静止
        gyro_norm = math.sqrt(gx_f*gx_f + gy_f*gy_f + gz_f*gz_f)
        is_static = (gyro_norm < 0.05 and residual_xy < 0.05)
        gz_comp = self.yawDriftComp.update_bias(gz_f, dt_s, is_static)

        # 6) 用 w_yaw 融合 原 gz_f 与 gz_comp，得到 gz_final
        gz_final = w_yaw * gz_comp + (1 - w_yaw) * gz_f

        # 7) 重新计算基于 gz_final 的增量四元数 (注意此时代替原 gz_f)
        dq_w, dq_x, dq_y, dq_z = self._gyro_to_quat_delta(gx_f, gy_f, gz_final, dt_s)
        # 与当前姿态相乘，得到新的预测姿态
        pred_w, pred_x, pred_y, pred_z = self._quat_multiply(
            self.q_w, self.q_x, self.q_y, self.q_z,
            dq_w, dq_x, dq_y, dq_z
        )

        # 8) 互补滤波：只修正 roll/pitch，不改 yaw
        fused_w, fused_x, fused_y, fused_z = self._complementary_accel(
            pred_w, pred_x, pred_y, pred_z,
            ax_f, ay_f, az_f
        )

        # 9) 归一化更新姿态
        qw, qx, qy, qz = self._normalize_quat(fused_w, fused_x, fused_y, fused_z)
        self.q_w, self.q_x, self.q_y, self.q_z = qw, qx, qy, qz

        return (qw, qx, qy, qz)

    def get_quaternion(self):
        """获取当前姿态四元数 (w, x, y, z)"""
        return (self.q_w, self.q_x, self.q_y, self.q_z)

    def get_euler_rad(self):
        """获取当前姿态欧拉角(roll, pitch, yaw)，单位：弧度"""
        return self._quat_to_euler(self.q_w, self.q_x, self.q_y, self.q_z)

    def get_euler_deg(self):
        """获取当前姿态欧拉角(roll, pitch, yaw)，单位：度"""
        roll, pitch, yaw = self.get_euler_rad()
        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

    # ----------------------- 私有辅助方法 -----------------------
    def _initialize_by_accel(self, ax, ay, az):
        """
        用加速度计数据计算roll, pitch; yaw置0，得到初始姿态
        """
        roll_rad, pitch_rad = self._accel_to_rp(ax, ay, az)
        yaw_rad = 0.0
        w, x, y, z = self._euler_to_quat(roll_rad, pitch_rad, yaw_rad)
        self.q_w, self.q_x, self.q_y, self.q_z = w, x, y, z
        self.initialized = True

    def _accel_to_rp(self, ax, ay, az):
        """
        从加速度数据估计roll, pitch:
          roll  = atan2(ay, az)
          pitch = atan2(-ax, sqrt(ay^2 + az^2))
        """
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        norm = max(norm, 1e-12)
        axn = ax / norm
        ayn = ay / norm
        azn = az / norm

        roll = math.atan2(ayn, azn)
        pitch = math.atan2(-axn, math.sqrt(ayn*ayn + azn*azn))
        return (roll, pitch)

    def _compute_gravity_ignore_yaw(self, roll, pitch):
        """
        计算当Yaw=0时，设备坐标系下的重力向量 (gpx, gpy, gpz)。
        即: Rz(0)*Ry(pitch)*Rx(roll) * [0, 0, 9.81]
        由于 Rz(0) = I, 实际仅需Ry(pitch)*Rx(roll)即可
        """
        g = 9.81
        # 先Rx(roll)作用到(0,0,g)
        sr = math.sin(roll)
        cr = math.cos(roll)
        gx1 = 0.0
        gy1 = -sr * g
        gz1 =  cr * g

        # 再对(gx1,gy1,gz1)应用Ry(pitch)
        sp = math.sin(pitch)
        cp = math.cos(pitch)
        # Ry(pitch)*[gx1, gy1, gz1]
        gpx = cp*gx1 + sp*gz1
        gpy = gy1
        gpz = -sp*gx1 + cp*gz1

        return (gpx, gpy, gpz)

    def _gyro_to_quat_delta(self, gx, gy, gz, dt_s):
        """
        根据陀螺仪角速度计算增量四元数，包含小角度泰勒展开优化。
        """
        omega_sq = gx*gx + gy*gy + gz*gz
        if omega_sq < 1e-8:
            # 小角度时，用泰勒展开简化，提高积分精度
            scale = 0.5 * dt_s * (1.0 - omega_sq * dt_s * dt_s / 24.0)
            return (1.0 - omega_sq * dt_s * dt_s / 8.0,
                    gx * scale,
                    gy * scale,
                    gz * scale)
        # 正常情况
        omega = math.sqrt(omega_sq)
        theta = omega * dt_s
        half_theta = 0.5 * theta
        sin_ht = math.sin(half_theta)
        cos_ht = math.cos(half_theta)
        s = sin_ht / omega
        return (cos_ht, gx*s, gy*s, gz*s)

    def _quat_multiply(self, w1, x1, y1, z1,
                             w2, x2, y2, z2):
        """
        四元数乘法 (Hamilton积), 乘后进行一次快速归一化
        """
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        mag = math.sqrt(w*w + x*x + y*y + z*z)
        mag = max(mag, 1e-12)
        return (w/mag, x/mag, y/mag, z/mag)

    def _normalize_quat(self, w, x, y, z):
        """
        归一化四元数，防止数值漂移
        """
        mag = math.sqrt(w*w + x*x + y*y + z*z)
        mag = max(mag, 1e-12)
        return (w/mag, x/mag, y/mag, z/mag)

    def _quat_to_euler(self, w, x, y, z):
        """
        四元数 -> 欧拉角 (roll, pitch, yaw)，返回弧度
        采用Z-Y-X顺序: yaw->pitch->roll。
        """
        mag = math.sqrt(w*w + x*x + y*y + z*z)
        mag = max(mag, 1e-12)
        w /= mag
        x /= mag
        y /= mag
        z /= mag

        # roll
        sinr_cosp = 2.0 * (w*x + y*z)
        cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch
        sinp = 2.0 * (w*y - z*x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi/2, sinp)
        else:
            pitch = math.asin(sinp)

        # yaw
        siny_cosp = 2.0 * (w*z + x*y)
        cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (roll, pitch, yaw)

    def _euler_to_quat(self, rollRad, pitchRad, yawRad):
        """
        欧拉角(Z-Y-X) => 四元数
        roll, pitch, yaw 均为弧度
        """
        cr = math.cos(0.5*rollRad)
        sr = math.sin(0.5*rollRad)
        cp = math.cos(0.5*pitchRad)
        sp = math.sin(0.5*pitchRad)
        cy = math.cos(0.5*yawRad)
        sy = math.sin(0.5*yawRad)

        q_w = cr*cp*cy + sr*sp*sy
        q_x = sr*cp*cy - cr*sp*sy
        q_y = cr*sp*cy + sr*cp*sy
        q_z = cr*cp*sy - sr*sp*cy
        return (q_w, q_x, q_y, q_z)

    def _complementary_accel(self, pred_w, pred_x, pred_y, pred_z,
                             ax_f, ay_f, az_f):
        """
        互补滤波融合:
          1) 解析预测四元数得到 pred_roll, pred_pitch, pred_yaw
          2) 加速度估算出 acc_roll, acc_pitch
          3) 根据加速度与重力9.81的偏差及其变化率计算acc_reliability
          4) 得到融合权重alpha, 融合roll/pitch (yaw 不变)
          5) 输出融合后的四元数

        此处不影响我们对yaw所做的ZUPT+Yaw约束，roll/pitch逻辑保持原有方式。
        """
        dt_s = self._last_dt_s if self._last_dt_s else 0.01

        # 预测姿态转欧拉角
        pred_roll, pred_pitch, pred_yaw = self._quat_to_euler(pred_w, pred_x, pred_y, pred_z)

        # 加速度估算roll, pitch
        acc_roll, acc_pitch = self._accel_to_rp(ax_f, ay_f, az_f)
        acc_norm = math.sqrt(ax_f*ax_f + ay_f*ay_f + az_f*az_f)
        diff_g = abs(acc_norm - 9.81)

        if self._last_acc_norm is not None:
            acc_change_rate = abs(acc_norm - self._last_acc_norm) / max(dt_s, 1e-12)
            # 综合考虑 diff_g 与 acc_change_rate
            acc_reliability = 1.0 - min(max(diff_g / 9.81, acc_change_rate / 20.0), 1.0)
        else:
            acc_reliability = 1.0 - min(diff_g / 9.81, 1.0)

        self._last_acc_norm = acc_norm

        alpha_min = 0.02
        alpha_max = 0.98
        alpha = alpha_min + (alpha_max - alpha_min)*(1.0 - acc_reliability)

        # 融合roll, pitch
        fused_roll  = alpha*pred_roll  + (1.0 - alpha)*acc_roll
        fused_pitch = alpha*pred_pitch + (1.0 - alpha)*acc_pitch
        fused_yaw   = pred_yaw  # yaw 不修正

        return self._euler_to_quat(fused_roll, fused_pitch, fused_yaw)

    @staticmethod
    def main_test():
        """
        使用本类的示例演示（单元测试）。
        """
        print("==== IMUAHRS Main Test: 演示姿态融合过程 ====")
        # 创建IMUAHRS对象，默认频率100Hz
        ahrs = IMUAHRS(default_freq=100.0)
        dt_us = 10000  # 10ms
        # 假设初始加速度(Z轴朝下9.81)，无角速度
        ax0, ay0, az0 = 0.0, 0.0, 9.81
        gx0, gy0, gz0 = 0.0, 0.0, 0.0

        print("---- First Update (Use Accel Init) ----")
        # 第一次更新时，会用加速度初始化姿态
        q_init = ahrs.update(ax0, ay0, az0, gx0, gy0, gz0, dt_us, temp_c=25.0)
        print("Init quaternion:", q_init)
        print("Init euler deg:", ahrs.get_euler_deg())

        # 模拟后续多帧：让陀螺仪有微小角速度 gx=0.01 rad/s
        for i in range(1, 6):
            gx_test = 0.01
            gy_test = 0.0
            gz_test = 0.0
            q_now = ahrs.update(ax0, ay0, az0, gx_test, gy_test, gz_test, dt_us, temp_c=30.0)
            rollDeg, pitchDeg, yawDeg = ahrs.get_euler_deg()
            print(f"Step={i}, quat=({q_now[0]:.4f}, {q_now[1]:.4f}, {q_now[2]:.4f}, {q_now[3]:.4f}), "
                  f"RPY=({rollDeg:.2f}, {pitchDeg:.2f}, {yawDeg:.2f})")


# =============================================================================
# 六、示例演示
# =============================================================================
if __name__ == "__main__":
    # 1) 先测试 YawDriftCompensator
    print("=== 测试 YawDriftCompensator ===")
    YawDriftCompensator.main_test()

    print("\n=== 测试 IMUAHRS ===")
    # 2) 测试 IMUAHRS
    IMUAHRS.main_test()
