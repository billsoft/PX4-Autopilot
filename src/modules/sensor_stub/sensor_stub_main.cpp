#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <unistd.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>

class SensorStub : public px4::ScheduledWorkItem {
public:
    SensorStub() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}
    bool init() {
        usleep(500000);
        _pub_accel0 = new uORB::PublicationMulti<sensor_accel_s>(ORB_ID(sensor_accel));
        _pub_accel1 = new uORB::PublicationMulti<sensor_accel_s>(ORB_ID(sensor_accel));
        _pub_gyro0  = new uORB::PublicationMulti<sensor_gyro_s>(ORB_ID(sensor_gyro));
        _pub_gyro1  = new uORB::PublicationMulti<sensor_gyro_s>(ORB_ID(sensor_gyro));
        _pub_mag = new uORB::PublicationMulti<sensor_mag_s>(ORB_ID(sensor_mag));
        ScheduleOnInterval(5000);
        return _pub_accel0 && _pub_accel1 && _pub_gyro0 && _pub_gyro1 && _pub_mag;
    }
    void Run() override {
        const uint64_t now = hrt_absolute_time();
        _tick++;
        PX4_INFO("sensor_stub tick %lu", (unsigned long)_tick);
        sensor_accel_s a{};
        a.timestamp = now;
        a.timestamp_sample = now;
        a.device_id = 0;
        a.x = (_tick % 20) < 10 ? 0.5f : -0.5f;
        a.y = (_tick % 15) < 7 ? 0.3f : -0.3f;
        a.z = 9.81f;
        if (_pub_accel0) { _pub_accel0->publish(a); }
        a.device_id = 1;
        a.x *= 0.8f;
        a.y *= 0.8f;
        if (_pub_accel1) { _pub_accel1->publish(a); }

        sensor_gyro_s g{};
        g.timestamp = now;
        g.timestamp_sample = now;
        g.device_id = 0;
        g.x = 0.0f; g.y = 0.0f; g.z = (_tick % 40) < 20 ? 0.05f : -0.05f;
        if (_pub_gyro0) { _pub_gyro0->publish(g); }
        g.device_id = 1;
        g.z *= 0.8f;
        if (_pub_gyro1) { _pub_gyro1->publish(g); }
        if ((_tick % 4) == 0) {
            sensor_mag_s m{};
            m.timestamp = now;
            m.device_id = 0;
            m.x = 0.1f;
            m.y = 0.0f;
            m.z = 0.4f;
            if (_pub_mag) { _pub_mag->publish(m); }
        }
    }
private:
    uORB::PublicationMulti<sensor_accel_s> *_pub_accel0{nullptr};
    uORB::PublicationMulti<sensor_accel_s> *_pub_accel1{nullptr};
    uORB::PublicationMulti<sensor_gyro_s>  *_pub_gyro0{nullptr};
    uORB::PublicationMulti<sensor_gyro_s>  *_pub_gyro1{nullptr};
    uORB::PublicationMulti<sensor_mag_s> *_pub_mag{nullptr};
    uint32_t _tick{0};
};

extern "C" int sensor_stub_main(int argc, char *argv[]) {
    static SensorStub *g{};
    if (!g) {
        g = new SensorStub();
        if (!g || !g->init()) { PX4_ERR("sensor_stub init fail"); return -1; }
    }
    g->ScheduleOnInterval(5000);
    g->ScheduleNow();
    PX4_INFO("sensor_stub started");
    return 0;
}
