#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_attitude.h>
#include <px4_platform_common/board_common.h>
#include <stm32_gpio.h>
#include <board_config.h>
#include <string.h>

class BoardStatusLEDs : public px4::ScheduledWorkItem {
public:
    BoardStatusLEDs() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default) {}
    ~BoardStatusLEDs() override = default;

    void setInvert(bool v) { _invert = v; }
    void setInvert1(bool v) { _invert1 = v; }
    void setInvert2(bool v) { _invert2 = v; }
    void setInvert3(bool v) { _invert3 = v; }
    void startTest(int secs) { uint64_t now = hrt_absolute_time(); _test_until_us = now + (uint64_t)secs * 1000000ULL; }

    bool init() {
        _accel0 = uORB::Subscription{ORB_ID(sensor_accel), 0};
        _accel1 = uORB::Subscription{ORB_ID(sensor_accel), 1};
        _mag = uORB::Subscription{ORB_ID(sensor_mag), 0};
        _att = uORB::Subscription{ORB_ID(vehicle_attitude), 0};
        _start_us = hrt_absolute_time();
        // ensure LEDs start OFF
        set_led(GPIO_nLED_GREEN, BOARD_LED_OFF);
        set_led(GPIO_nLED_YELLOW, BOARD_LED_OFF);
        set_led(GPIO_nLED_RED, BOARD_LED_OFF);
        ScheduleOnInterval(100000);
        return true;
    }

    void Run() override {
        const uint64_t now = hrt_absolute_time();
        sensor_accel_s a0{}; sensor_accel_s a1{}; sensor_mag_s m{}; vehicle_attitude_s va{};
        bool imu1 = false, imu2 = false, mag = false, fusion = false;
        if (_accel0.updated()) { _accel0.copy(&a0); _t_accel0 = a0.timestamp; }
        if (_accel1.updated()) { _accel1.copy(&a1); _t_accel1 = a1.timestamp; }
        if (_mag.updated()) { _mag.copy(&m); _t_mag = m.timestamp; }
        if (_att.updated()) { _att.copy(&va); _t_att = va.timestamp; }
        imu1 = (now - _t_accel0) < _window_us;
        imu2 = (now - _t_accel1) < _window_us;
        mag  = (now - _t_mag) < _window_us;
        fusion = (now - _t_att) < _window_us;
        _tick++;
        bool blink_slow_g = ((_tick + 0) % 20) < 10;
        bool blink_slow_y = ((_tick + 7) % 20) < 10;
        bool blink_slow_r = ((_tick + 14) % 20) < 10;
        bool blink_fast2 = (_tick % 5) < 2;
        bool blink_fusion = (_tick % 3) < 1;

        // Test override
        if (_test_until_us > now) {
            uint32_t s = (_tick % 6);
            set_led(GPIO_nLED_GREEN, (s < 2) ? BOARD_LED_ON : BOARD_LED_OFF);
            set_led(GPIO_nLED_YELLOW, (s >= 2 && s < 4) ? BOARD_LED_ON : BOARD_LED_OFF);
            set_led(GPIO_nLED_RED, (s >= 4) ? BOARD_LED_ON : BOARD_LED_OFF);
            return;
        }

        // Simple startup indication (first 3s)
        const bool heartbeat = (now - _start_us) < 3000000;
        if (heartbeat) {
             // Slow blink all to show system is alive
            set_led(GPIO_nLED_GREEN, blink_slow_g ? BOARD_LED_ON : BOARD_LED_OFF);
            set_led(GPIO_nLED_YELLOW, blink_slow_y ? BOARD_LED_ON : BOARD_LED_OFF);
            set_led(GPIO_nLED_RED, blink_slow_r ? BOARD_LED_ON : BOARD_LED_OFF);
            return;
        }

        // Independent logic per sensor: Fast = Data, Slow = No Data
        // Fusion indication: LED1+LED2 simultaneous faster blink (~3.3Hz) overrides normal state

        // LED1 (Green) - IMU1 (SPI1)
        if (fusion) {
            set_led(GPIO_nLED_GREEN, blink_fusion ? BOARD_LED_ON : BOARD_LED_OFF);
        } else if (imu1) {
            set_led(GPIO_nLED_GREEN, blink_fast2 ? BOARD_LED_ON : BOARD_LED_OFF);
        } else {
            set_led(GPIO_nLED_GREEN, blink_slow_g ? BOARD_LED_ON : BOARD_LED_OFF);
        }

        // LED2 (Yellow) - IMU2 (SPI3)
        if (fusion) {
            set_led(GPIO_nLED_YELLOW, blink_fusion ? BOARD_LED_ON : BOARD_LED_OFF);
        } else if (imu2) {
            set_led(GPIO_nLED_YELLOW, blink_fast2 ? BOARD_LED_ON : BOARD_LED_OFF);
        } else {
            set_led(GPIO_nLED_YELLOW, blink_slow_y ? BOARD_LED_ON : BOARD_LED_OFF);
        }

        // LED3 (Red) - Mag (I2C1)
        if (mag) {
            set_led(GPIO_nLED_RED, blink_fast2 ? BOARD_LED_ON : BOARD_LED_OFF);
        } else {
            set_led(GPIO_nLED_RED, blink_slow_r ? BOARD_LED_ON : BOARD_LED_OFF);
        }

        if (now - _last_log_us > 5000000) {
            PX4_INFO("leds tick=%lu imu1=%d(%llu) imu2=%d(%llu) mag=%d(%llu) fusion=%d(%llu)",
                (unsigned long)_tick,
                (int)imu1, (unsigned long long)(now - _t_accel0),
                (int)imu2, (unsigned long long)(now - _t_accel1),
                (int)mag, (unsigned long long)(now - _t_mag),
                (int)fusion, (unsigned long long)(now - _t_att));
            _last_log_us = now;
        }
    }

private:
    void set_led(uint32_t gpio, int onoff) {
        int v = onoff;
        // Apply global invert
        if (_invert) {
            v = (v == BOARD_LED_ON) ? BOARD_LED_OFF : BOARD_LED_ON;
        }
        // Apply per-channel invert
        if (gpio == GPIO_nLED_GREEN && _invert1) {
            v = (v == BOARD_LED_ON) ? BOARD_LED_OFF : BOARD_LED_ON;
        }
        if (gpio == GPIO_nLED_YELLOW && _invert2) {
            v = (v == BOARD_LED_ON) ? BOARD_LED_OFF : BOARD_LED_ON;
        }
        if (gpio == GPIO_nLED_RED && _invert3) {
            v = (v == BOARD_LED_ON) ? BOARD_LED_OFF : BOARD_LED_ON;
        }
        px4_arch_gpiowrite(gpio, v);
    }
    uORB::Subscription _accel0{ORB_ID(sensor_accel), 0};
    uORB::Subscription _accel1{ORB_ID(sensor_accel), 1};
    uORB::Subscription _mag{ORB_ID(sensor_mag), 0};
    uORB::Subscription _att{ORB_ID(vehicle_attitude), 0};
    uint64_t _t_accel0{0}, _t_accel1{0}, _t_mag{0}, _t_att{0};
    uint64_t _window_us{500000};
    uint32_t _tick{0};
    uint64_t _start_us{0};
    bool _invert{false};
    bool _invert1{false};
    bool _invert2{false};
    bool _invert3{false};
    uint64_t _test_until_us{0};
    uint64_t _last_log_us{0};
};

extern "C" __EXPORT int board_status_leds_main(int argc, char *argv[]) {
    static BoardStatusLEDs *g_app = nullptr;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-invert") == 0) {
            if (g_app) { g_app->setInvert(true); }
            // if not created yet, remember to set after construction below
        }
        if (strcmp(argv[i], "-invert1") == 0) { if (g_app) { g_app->setInvert1(true); } }
        if (strcmp(argv[i], "-invert2") == 0) { if (g_app) { g_app->setInvert2(true); } }
        if (strcmp(argv[i], "-invert3") == 0) { if (g_app) { g_app->setInvert3(true); } }
    }
    if (!g_app) {
        g_app = new BoardStatusLEDs();
        for (int i = 1; i < argc; i++) { if (strcmp(argv[i], "-invert") == 0) g_app->setInvert(true); }
        for (int i = 1; i < argc; i++) { if (strcmp(argv[i], "-invert1") == 0) g_app->setInvert1(true); }
        for (int i = 1; i < argc; i++) { if (strcmp(argv[i], "-invert2") == 0) g_app->setInvert2(true); }
        for (int i = 1; i < argc; i++) { if (strcmp(argv[i], "-invert3") == 0) g_app->setInvert3(true); }
        if (!g_app->init()) { PX4_ERR("init fail"); delete g_app; g_app = nullptr; return -1; }
    }
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "test") == 0) {
            int secs = 5;
            if (i + 1 < argc) { secs = atoi(argv[i + 1]); }
            g_app->startTest(secs);
        }
    }
    // ensure scheduled
    g_app->ScheduleNow();
    return 0;
}

