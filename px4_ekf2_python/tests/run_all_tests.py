"""
PX4 EKF2 - 完整测试套件运行脚本
按顺序执行：数据生成 → 单元测试 → 集成测试
"""
import sys
import os
import time

# 添加路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def print_header(title):
    """打印标题"""
    print("\n" + "="*70)
    print(f"  {title}")
    print("="*70)


def run_step(step_name, func):
    """运行测试步骤"""
    print_header(f"步骤: {step_name}")
    start_time = time.time()

    try:
        result = func()
        elapsed = time.time() - start_time

        if result:
            print(f"\n✅ {step_name} 成功 (耗时: {elapsed:.2f}秒)")
            return True
        else:
            print(f"\n❌ {step_name} 失败")
            return False

    except Exception as e:
        print(f"\n❌ {step_name} 错误: {e}")
        import traceback
        traceback.print_exc()
        return False


def step1_generate_data():
    """步骤1: 生成测试数据"""
    from test_data_generator import generate_test_dataset

    # 悬停场景
    dataset_hover = generate_test_dataset(
        scenario='hover',
        duration=10.0,
        save_path='tests/data/hover_scenario.pkl'
    )

    # 前飞场景
    dataset_forward = generate_test_dataset(
        scenario='forward_flight',
        duration=20.0,
        save_path='tests/data/forward_flight_scenario.pkl'
    )

    return dataset_hover is not None and dataset_forward is not None


def step2_unit_tests():
    """步骤2: 运行单元测试"""
    from test_unit import run_all_unit_tests
    return run_all_unit_tests()


def step3_integration_tests():
    """步骤3: 运行集成测试"""
    from test_integration import run_integration_test

    # 测试两个场景
    success1 = run_integration_test('hover')
    success2 = run_integration_test('forward_flight')

    return success1 and success2


def main():
    """主测试流程"""
    print("\n" + "#"*70)
    print("#" + " "*68 + "#")
    print("#" + "  PX4 EKF2 Python - 完整测试套件  ".center(68) + "#")
    print("#" + " "*68 + "#")
    print("#"*70)

    overall_start = time.time()

    # 测试步骤
    steps = [
        ("数据生成", step1_generate_data),
        ("单元测试", step2_unit_tests),
        ("集成测试", step3_integration_tests)
    ]

    results = []

    for step_name, step_func in steps:
        result = run_step(step_name, step_func)
        results.append((step_name, result))

        if not result:
            print(f"\n⚠️  {step_name}失败，停止后续测试")
            break

    # 总结
    overall_elapsed = time.time() - overall_start

    print("\n" + "="*70)
    print("  测试总结")
    print("="*70)

    for step_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"  {step_name:20s} {status}")

    all_passed = all(result for _, result in results)

    print("\n" + "="*70)
    if all_passed:
        print("  🎉 所有测试通过！")
    else:
        print("  ⚠️  部分测试失败")

    print(f"  总耗时: {overall_elapsed:.2f}秒")
    print("="*70 + "\n")

    # 结果文件位置
    if all_passed:
        print("结果文件:")
        print("  - tests/hover_results.png")
        print("  - tests/forward_flight_results.png")
        print("  - tests/data/hover_scenario.pkl")
        print("  - tests/data/forward_flight_scenario.pkl")

    return 0 if all_passed else 1


if __name__ == '__main__':
    exit_code = main()
    sys.exit(exit_code)
