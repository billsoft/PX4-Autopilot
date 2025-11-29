#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/gpio_in.h>
#include <board_config.h>

using namespace time_literals;

class CmosSync : public ModuleBase<CmosSync>, public ModuleParams, public px4::ScheduledWorkItem {
public:
    CmosSync() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default) {}

    static int task_spawn(int argc, char *argv[]) {
        CmosSync *instance = new CmosSync();

        if (instance) {
            _object.store(instance);
            _task_id = task_id_is_work_queue;

            if (instance->init()) {
                return PX4_OK;
            }
        } else {
            PX4_ERR("alloc failed");
        }

        delete instance;
        _object.store(nullptr);
        _task_id = -1;
        return PX4_ERROR;
    }

    static int print_usage(const char *reason = nullptr) {
        if (reason) { PX4_WARN("%s\n", reason); }
        PRINT_MODULE_DESCRIPTION("CMOS sync polling and timestamp publisher");
        PRINT_MODULE_USAGE_NAME("cmos_sync", "driver");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
        return 0;
    }

    static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }

    bool init() {
        _last_state = read_state();
        ScheduleOnInterval(_interval_us);
        return true;
    }

private:
    void Run() override {
        if (should_exit()) {
            exit_and_cleanup();
            return;
        }

        const uint32_t state = read_state();
        const uint32_t changed = state ^ _last_state;

        // simple debounce: require minimum interval between publications
        const hrt_abstime now = hrt_absolute_time();
        if (changed && (now - _last_pub_ts) > _debounce_us) {
            gpio_in_s msg{};
            msg.timestamp = now;
            msg.device_id = 0;
            msg.state = state;
            _pub.publish(msg);
            _last_pub_ts = now;
            _last_state = state;
        }
    }

    uint32_t read_state() const {
        uint32_t mask = 0;
        if (px4_arch_gpioread(GPIO_CMOS_SYNC_LINE)) { mask |= (1u << 0); }
        if (px4_arch_gpioread(GPIO_CMOS_SYNC_FRAME)) { mask |= (1u << 1); }
        return mask;
    }

    uORB::Publication<gpio_in_s> _pub{ORB_ID(gpio_in)};
    uint32_t _last_state{0};
    hrt_abstime _last_pub_ts{0};
    uint32_t _interval_us{1000};
    uint32_t _debounce_us{500};
};

extern "C" __EXPORT int cmos_sync_main(int argc, char *argv[]) { return CmosSync::main(argc, argv); }

