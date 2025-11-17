"""
PX4 EKF2 单元测试
测试各个模块的功能
"""
import numpy as np
import sys
import os

# 添加父目录到路径
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, parent_dir)

# 绝对导入
import utils
import imu_processor
import state
import ekf_core

Quaternion = utils.Quaternion
WelfordOnlineVariance = utils.WelfordOnlineVariance
skew_symmetric = utils.skew_symmetric
IMUProcessor = imu_processor.IMUProcessor
ConingIntegrator = imu_processor.ConingIntegrator
StateVector = state.StateVector
Parameters = state.Parameters
EKF2Core = ekf_core.EKF2Core


def test_quaternion():
    """测试四元数运算"""
    print("\n" + "="*60)
    print("测试: 四元数运算")
    print("="*60)

    # 测试1: 单位四元数
    q = Quaternion()
    assert np.allclose(q.q, [1, 0, 0, 0]), "单位四元数错误"
    print("✓ 单位四元数正确")

    # 测试2: 轴角转换
    axis = np.array([0, 0, 1])  # Z轴
    angle = np.pi / 2  # 90°
    q = Quaternion.from_axis_angle(axis, angle)
    R = q.to_rotation_matrix()

    # 90°绕Z轴旋转: X→Y, Y→-X
    v_in = np.array([1, 0, 0])
    v_out = R @ v_in
    assert np.allclose(v_out, [0, 1, 0], atol=1e-6), "旋转矩阵错误"
    print("✓ 轴角转四元数正确")

    # 测试3: 四元数乘法
    q1 = Quaternion.from_axis_angle(np.array([0, 0, 1]), np.pi/4)  # 45°
    q2 = Quaternion.from_axis_angle(np.array([0, 0, 1]), np.pi/4)  # 45°
    q3 = q1 * q2  # 应该=90°

    roll, pitch, yaw = q3.to_euler()
    assert np.allclose(yaw, np.pi/2, atol=1e-6), "四元数乘法错误"
    print("✓ 四元数乘法正确")

    # 测试4: 归一化
    q = Quaternion(1, 1, 1, 1)
    q = q.normalized()
    assert np.allclose(np.linalg.norm(q.q), 1.0), "归一化错误"
    print("✓ 四元数归一化正确")

    print("\n四元数测试通过 ✅\n")


def test_welford_variance():
    """测试Welford在线方差算法"""
    print("="*60)
    print("测试: Welford在线方差")
    print("="*60)

    # 生成测试数据
    np.random.seed(42)
    data = np.random.normal(10, 2, 1000)  # 均值10, 标准差2

    # Welford算法
    wv = WelfordOnlineVariance()
    for x in data:
        wv.update(x)

    # 对比numpy结果
    mean_true = np.mean(data)
    std_true = np.std(data, ddof=0)

    print(f"均值: Welford={wv.mean:.4f}, NumPy={mean_true:.4f}")
    print(f"标准差: Welford={wv.get_std():.4f}, NumPy={std_true:.4f}")

    assert np.allclose(wv.mean, mean_true, rtol=1e-4), "均值计算错误"
    assert np.allclose(wv.get_std(), std_true, rtol=1e-4), "标准差计算错误"

    print("\nWelford方差测试通过 ✅\n")


def test_coning_compensation():
    """测试圆锥补偿"""
    print("="*60)
    print("测试: 圆锥补偿")
    print("="*60)

    integrator = ConingIntegrator()

    # 模拟真实圆锥运动: 旋转轴缓慢进动
    dt = 0.001
    theta_total = np.zeros(3)
    for k in range(100):
        gyro = np.array([np.sin(0.1 * k), np.cos(0.1 * k), 0.0])
        theta_total = integrator.integrate(gyro, dt)

    # 检查补偿效果（应该有Z轴分量）
    print(f"积分结果: {theta_total}")
    print(f"Z轴分量（圆锥误差补偿）: {theta_total[2]:.6f} rad")

    assert abs(theta_total[2]) > 1e-6, "圆锥补偿未生效"

    print("\n圆锥补偿测试通过 ✅\n")


def test_imu_processor():
    """测试IMU处理器"""
    print("="*60)
    print("测试: IMU处理器")
    print("="*60)

    proc = IMUProcessor(target_dt=0.004)  # 250Hz输出

    # 模拟高频IMU输入（1kHz）
    dt = 0.001
    imu_samples = []

    for i in range(10):  # 10ms数据
        gyro = np.array([0.1, 0.0, 0.0])  # 绕X轴
        accel = np.array([0, 0, -9.81])  # 静止

        sample = proc.update(gyro, accel, dt)

        if sample:
            imu_samples.append(sample)
            print(f"  输出样本 {len(imu_samples)}: "
                  f"delta_ang={np.linalg.norm(sample.delta_ang):.6f} rad, "
                  f"dt={sample.delta_ang_dt:.4f} s")

    # 应该输出2-3个样本（10ms / 4ms）
    assert len(imu_samples) >= 2, "IMU积分输出错误"

    # 检查积分时间
    for sample in imu_samples:
        assert np.isclose(sample.delta_ang_dt, 0.004, atol=0.001), \
            "积分时间错误"

    print(f"\n共输出 {len(imu_samples)} 个IMU样本")
    print("IMU处理器测试通过 ✅\n")


def test_state_prediction():
    """测试状态预测"""
    print("="*60)
    print("测试: 状态预测")
    print("="*60)

    ekf = EKF2Core()

    # 初始状态: 水平静止
    ekf.state.quat = Quaternion()
    ekf.state.vel = np.zeros(3)
    ekf.state.pos = np.zeros(3)

    # IMU输入: 匀速向上运动
    dt = 0.01
    imu = state.IMUSample(
        delta_ang=np.zeros(3),  # 无旋转
        delta_vel=np.array([0, 0, -12*dt]),  # 向上加速（NED-Down为负）
        delta_ang_dt=dt,
        delta_vel_dt=dt
    )

    # 预测100步（1秒）
    for _ in range(100):
        ekf.predict_state(imu)

    print(f"1秒后速度: {ekf.state.vel}")
    print(f"1秒后位置: {ekf.state.pos}")

    # 向上加速度 = 12 - 9.81 = 2.19 m/s²
    # 1秒后速度约 -2.19 m/s (向上)
    expected_vel_z = -(12 - ekf.params.gravity) * 1.0
    assert np.allclose(ekf.state.vel[2], expected_vel_z, rtol=0.1), \
        f"速度预测错误: {ekf.state.vel[2]} vs {expected_vel_z}"

    print("\n状态预测测试通过 ✅\n")


def test_coriolis_correction():
    """测试科里奥利修正"""
    print("="*60)
    print("测试: 科里奥利修正")
    print("="*60)

    params = Parameters()
    params.latitude = np.radians(45)  # 45°N
    params.enable_coriolis_correction = True

    ekf = EKF2Core(params)

    # 设置初始速度: 100 m/s 北向
    ekf.state.vel = np.array([-100, 0, 0])

    # IMU输入: 无加速度（匀速）
    dt = 0.01
    imu = state.IMUSample(
        delta_ang=np.zeros(3),
        delta_vel=np.array([0, 0, -ekf.params.gravity*dt]),  # 仅重力
        delta_ang_dt=dt,
        delta_vel_dt=dt
    )

    # 运行10秒
    for _ in range(1000):
        ekf.predict_state(imu)

    print(f"10秒后速度: {ekf.state.vel}")
    print(f"东向速度分量: {ekf.state.vel[1]:.4f} m/s")

    # 科里奥利力应该产生东向速度分量
    # Ω_N = Ω * cos(45°) ≈ 5.16e-5 rad/s
    # a_E = 2 * Ω_N * v_N ≈ 0.01 m/s²
    # 10秒后: v_E ≈ 0.1 m/s
    assert abs(ekf.state.vel[1]) > 0.05, "科里奥利修正未生效"

    print("\n科里奥利修正测试通过 ✅\n")


def test_bias_inhibit():
    """测试偏置学习抑制"""
    print("="*60)
    print("测试: 轴向选择性偏置抑制")
    print("="*60)

    ekf = EKF2Core()
    ekf.control_status['gravity_vector_fusion'] = True

    # 模拟高机动IMU数据
    dt = 0.01
    imu = state.IMUSample(
        delta_ang=np.array([0.1, 0, 0]) * dt,  # 高角速度
        delta_vel=np.array([30, 0, 0]) * dt,   # 高加速度
        delta_ang_dt=dt,
        delta_vel_dt=dt
    )

    # 更新机动检测
    ekf._update_maneuver_detection(imu)

    print(f"高机动标志: {ekf.fault_status['high_maneuver']}")
    print(f"陀螺偏置抑制: {ekf.gyro_bias_inhibit}")
    print(f"加速度偏置抑制: {ekf.accel_bias_inhibit}")

    # 高机动应该触发全轴抑制
    assert ekf.fault_status['high_maneuver'], "高机动检测失败"
    assert ekf.gyro_bias_inhibit, "陀螺偏置抑制失败"
    assert np.all(ekf.accel_bias_inhibit), "加速度偏置抑制失败"

    print("\n偏置抑制测试通过 ✅\n")


def run_all_unit_tests():
    """运行所有单元测试"""
    print("\n" + "#"*60)
    print("# PX4 EKF2 Python - 单元测试套件")
    print("#"*60)

    try:
        test_quaternion()
        test_welford_variance()
        test_coning_compensation()
        test_imu_processor()
        test_state_prediction()
        test_coriolis_correction()
        test_bias_inhibit()

        print("\n" + "="*60)
        print("所有单元测试通过 ✅✅✅")
        print("="*60)
        return True

    except AssertionError as e:
        print(f"\n❌ 测试失败: {e}")
        return False
    except Exception as e:
        print(f"\n❌ 测试错误: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == '__main__':
    success = run_all_unit_tests()
    sys.exit(0 if success else 1)
