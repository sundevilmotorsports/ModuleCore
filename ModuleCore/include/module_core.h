#pragma once

#include "esp_err.h"
#include "esp_ota_ops.h"
#include "esp_twai.h"
#include "esp_twai_types.h"
#include "esp_twai_onchip.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <cstdint>
#include <functional>
#include <expected>
#include "error.h"
#include "esp_partition.h"

#define CAN_NVS_KEY   "can_id"
#define CAN_ID_UNSET  0xFF
#define CAN_BROADCAST 0x7FF

// ── Protocol ─────────────────────────────────────────────────────────────────
//
// Every node runs ModuleCore. Any node can act as a bridge (UART host → CAN)
// or as an OTA target (receives its own firmware over CAN).
//
// UART framing  →  [target(1), cmd(1), payload...]
//
// OTA UART commands (target = node to update):
//   CMD_OTA_DATA   [target, 0x06, seq_hi, seq_lo, data(1-5)]
//   CMD_OTA_ABORT  [target, 0x07]
//
// OTA CAN frames (bridge → target):
//   OTA_DATA   [0xF0, seq_hi, seq_lo, data(1-5)]   first frame (seq=0) begins OTA
//   OTA_END    [0xF1]                               commit + reboot
//   OTA_ABORT  [0xF2]                               abort
//
// OTA CAN frames (target → bridge):
//   OTA_ACK    [0xF3, seq_hi, seq_lo]
//   OTA_NAK    [0xF4, seq_hi, seq_lo]
//   OTA_DONE   [0xF5, result]                       result 0=ok, sent before reboot
//
// UART status responses (bridge → host):
//   [MSG_OTA_STATUS, target_id, status]
//   status: 0=progress, 1=done_ok, 2=done_fail, 3=aborted
//
// ─────────────────────────────────────────────────────────────────────────────

enum Command : uint8_t {
    CMD_IDENTIFY  = 0x01,
    CMD_SET_ID    = 0x02,
    CMD_GET_INFO  = 0x03,
    CMD_DISCOVER  = 0x04,
    CMD_OTA_DATA  = 0x06,
    CMD_OTA_ABORT = 0x07,
};

enum MsgType : uint8_t {
    MSG_RESPONSE   = 0x10,
    MSG_DISCOVERY  = 0x11,
    MSG_OTA_STATUS = 0x12,
};

enum OtaCanCmd : uint8_t {
    OTA_DATA  = 0xF0,
    OTA_END   = 0xF1,
    OTA_ABORT = 0xF2,
    OTA_ACK   = 0xF3,
    OTA_NAK   = 0xF4,
    OTA_DONE  = 0xF5,
};

enum OtaStatus : uint8_t {
    OTA_STATUS_PROGRESS  = 0x00,
    OTA_STATUS_DONE_OK   = 0x01,
    OTA_STATUS_DONE_FAIL = 0x02,
    OTA_STATUS_ABORTED   = 0x03,
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

        uint32_t ota_ack_timeout_ms = 500;
        uint8_t  ota_max_retries    = 3;
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

    // Bridge-side: tracks an outgoing OTA session to a remote node
    struct BridgeOta {
        uint8_t           target_id  = 0;
        uint16_t          next_seq   = 0;
        bool              active     = false;
        bool              ack_ok     = false;
        SemaphoreHandle_t ack_sem    = nullptr;
    };

    // Target-side: tracks an incoming OTA being written to local flash
    struct TargetOta {
        esp_ota_handle_t        handle     = 0;
        const esp_partition_t  *partition  = nullptr;
        uint16_t                next_seq   = 0;
        bool                    active     = false;
    };

    template<void (ModuleCore::*Method)()>
    void spawnTask(const char *name, uint32_t stack, UBaseType_t prio) {
        xTaskCreate([](void *arg) {
            (static_cast<ModuleCore*>(arg)->*Method)();
            vTaskDelete(nullptr);
        }, name, stack, this, prio, nullptr);
    }

    // Core
    esp_err_t handleUart(const uint8_t *data, size_t len);
    bool      handleCan(const CanFrame *frame);
    esp_err_t sendUartResponse(const UartResponse &resp);
    esp_err_t setId(uint8_t id);
    esp_err_t startDiscovery();
    void      identify();
    uint8_t   loadId();
    uint8_t   getId() const { return can_id_; }

    // OTA bridge
    esp_err_t bridgeSendChunk(uint8_t target, uint16_t seq, const uint8_t *data, size_t len, bool last);
    esp_err_t bridgeAbort();
    void      bridgeHandleAck(const CanFrame *frame);

    // OTA target
    void targetHandleData(const CanFrame *frame);
    void targetHandleEnd();
    void targetHandleAbort();

    // Tasks
    void identifyTask();
    void discoveryTask();
    void canProcessTask();
    void uartRxLoop();
    void appSupervisorTask();

    static bool onCanRxStatic(twai_node_handle_t handle,
                               const twai_rx_done_event_data_t *edata, void *ctx);
    static void uartRxTaskEntry(void *arg);

    twai_node_handle_t twai_hdl_         = nullptr;
    uart_port_t        uart_port_        = UART_NUM_0;
    ModuleInfo         info_             = {};
    Config             cfg_              = {};
    uint8_t            can_id_           = CAN_ID_UNSET;
    bool               discovery_active_ = false;
    QueueHandle_t      can_queue_        = nullptr;

    BridgeOta          bridge_ota_       = {};
    TargetOta          target_ota_       = {};
};
