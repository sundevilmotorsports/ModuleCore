#pragma once

#include "esp_err.h"
#include "esp_twai.h"
#include "esp_twai_types.h"
#include "esp_twai_onchip.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <cstdint>
#include <functional>
#include <expected>
#include "error.h"

#define CAN_NVS_KEY   "can_id"
#define CAN_ID_UNSET  0xFF
#define CAN_BROADCAST 0x7FF

enum Command : uint8_t {
    CMD_IDENTIFY = 0x01,
    CMD_SET_ID   = 0x02,
    CMD_GET_INFO = 0x03,
    CMD_DISCOVER = 0x04,
};

enum MsgType : uint8_t {
    MSG_RESPONSE  = 0x10,
    MSG_DISCOVERY = 0x11,
};

struct ModuleInfo {
    uint8_t    module_type;
    uint8_t    fw_version;
    gpio_num_t blink_pin;
};

struct CanFrame {
    twai_frame_header_t header;
    uint8_t             data[8];
    uint8_t             len;
};

class ModuleCore {
public:
    struct Config {
        gpio_num_t                 can_tx;
        gpio_num_t                 can_rx;
        uart_port_t                uart_port  = UART_NUM_0;
        int                        uart_baud  = 115200;
        twai_timing_basic_config_t can_timing = { .bitrate = 1000000 };

        std::function<void(const CanFrame*)>        on_can_rx  = nullptr;
        std::function<void(const uint8_t*, size_t)> on_uart_rx = nullptr;

        std::function<std::expected<void, ModuleCoreError>()> app_main = nullptr;
    };

    ModuleCore() = default;
    ~ModuleCore();

    esp_err_t init(const ModuleInfo &info, const Config &cfg);
    esp_err_t sendCanFrame(uint32_t can_id, const uint8_t *data, size_t len);

private:
    struct UartMessage {
        uint8_t target_device;
        uint8_t command;
        uint8_t payload[6];
        size_t  payload_len;
    };

    struct UartResponse {
        MsgType msg_type;
        uint8_t source_device;
        uint8_t data[6];
        size_t  data_len;
    };

    template<void (ModuleCore::*Method)()>
    void spawnTask(const char *name, uint32_t stack, UBaseType_t prio) {
        xTaskCreate([](void *arg) {
            (static_cast<ModuleCore*>(arg)->*Method)();
            vTaskDelete(nullptr);
        }, name, stack, this, prio, nullptr);
    }

    esp_err_t handleUart(const uint8_t *data, size_t len);
    bool      handleCan(const CanFrame *frame);
    esp_err_t sendUartResponse(const UartResponse &resp);
    esp_err_t setId(uint8_t id);
    esp_err_t startDiscovery();
    void      identify();
    uint8_t   loadId();
    uint8_t   getId() const { return can_id_; }

    void identifyTask();
    void discoveryTask();
    void canProcessTask();
    void uartRxLoop();

    void appSupervisorTask();

    static bool onCanRxStatic(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *ctx);
    static void uartRxTaskEntry(void *arg);

    twai_node_handle_t twai_hdl_         = nullptr;
    uart_port_t        uart_port_        = UART_NUM_0;
    ModuleInfo         info_             = {};
    Config             cfg_              = {};
    uint8_t            can_id_           = CAN_ID_UNSET;
    bool               discovery_active_ = false;
    QueueHandle_t      can_queue_        = nullptr;
};
