@echo off
REM PX4 EKF2 Python测试运行脚本
echo ======================================================================
echo PX4 EKF2 Python - 测试套件
echo ======================================================================

cd /d %~dp0

REM Step 1: 生成测试数据
echo.
echo Step 1: 生成测试数据...
python -m tests.test_data_generator
if %ERRORLEVEL% neq 0 (
    echo 数据生成失败
    pause
    exit /b 1
)

REM Step 2: 运行单元测试
echo.
echo Step 2: 运行单元测试...
python -m tests.test_unit
if %ERRORLEVEL% neq 0 (
    echo 单元测试失败
    pause
    exit /b 1
)

REM Step 3: 运行集成测试
echo.
echo Step 3: 运行集成测试...
python -m tests.test_integration
if %ERRORLEVEL% neq 0 (
    echo 集成测试失败
    pause
    exit /b 1
)

echo.
echo ======================================================================
echo 所有测试通过！
echo ======================================================================
pause
