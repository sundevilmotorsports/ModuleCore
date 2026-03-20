#include "module_core.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_ota_ops.h"
#include "nvs.h"
#include <cstring>

static const char *TAG = "ModuleCore";

// ── Lifecycle ─────────────────────────────────────────────────────────────────

ModuleCore::~ModuleCore() {
    if (bridge_ota_.active) bridgeAbort();
    if (target_ota_.active) targetHandleAbort();

    if (twai_hdl_) {
        twai_node_disable(twai_hdl_);
        twai_node_delete(twai_hdl_);
    }
    uart_driver_delete(uart_port_);
    if (can_queue_) vQueueDelete(can_queue_);
}

esp_err_t ModuleCore::init(const ModuleInfo &info, const Config &cfg) {
    info_      = info;
    cfg_       = cfg;
    uart_port_ = cfg.uart_port;
    can_id_    = loadId();

    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate  = cfg.uart_baud;
    uart_cfg.data_bits  = UART_DATA_8_BITS;
    uart_cfg.parity     = UART_PARITY_DISABLE;
    uart_cfg.stop_bits  = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    ESP_RETURN_ON_ERROR(uart_param_config(uart_port_, &uart_cfg),
                        TAG, "UART param config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(uart_port_,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "UART set pin failed");
    ESP_RETURN_ON_ERROR(uart_driver_install(uart_port_, 512, 512, 0, nullptr, 0),
                        TAG, "UART install failed");

    can_queue_ = xQueueCreate(16, sizeof(CanFrame));

    // twai_onchip_node_config_t node_cfg = {};
    // node_cfg.io_cfg.tx                = cfg.can_tx;
    // node_cfg.io_cfg.rx                = cfg.can_rx;
    // node_cfg.io_cfg.quanta_clk_out    = GPIO_NUM_NC;
    // node_cfg.io_cfg.bus_off_indicator = GPIO_NUM_NC;
    // node_cfg.bit_timing               = cfg.can_timing;
    // node_cfg.fail_retry_cnt           = 3;
    // node_cfg.tx_queue_depth           = 16;
    //
    // ESP_RETURN_ON_ERROR(twai_new_node_onchip(&node_cfg, &twai_hdl_),
    //                     TAG, "TWAI init failed");
    //
    // twai_event_callbacks_t cbs = {};
    // cbs.on_rx_done = onCanRxStatic;
    // ESP_RETURN_ON_ERROR(twai_node_register_event_callbacks(twai_hdl_, &cbs, this),
    //                     TAG, "TWAI cb register failed");
    //
    // ESP_RETURN_ON_ERROR(twai_node_enable(twai_hdl_), TAG, "TWAI enable failed");
    //
    // spawnTask<&ModuleCore::canProcessTask>("can_proc", 4096, 6);

    xTaskCreate(uartRxTaskEntry, "uart_rx", 4096, this, 5, nullptr);

    ESP_LOGI(TAG, "Ready — CAN ID: 0x%02X", getId());

    if (cfg_.app_main) {
        spawnTask<&ModuleCore::appSupervisorTask>("app_sup", 8192, 5);
    } else {
        ESP_LOGE(TAG, "No app_main provided");
    }

    return ESP_OK;
}

// ── CAN ISR + task ────────────────────────────────────────────────────────────

bool ModuleCore::onCanRxStatic(twai_node_handle_t handle,
                                const twai_rx_done_event_data_t *edata, void *ctx) {
    auto *self = static_cast<ModuleCore *>(ctx);

    uint8_t recv_buff[8];
    twai_frame_t rx_frame = { .buffer = recv_buff, .buffer_len = sizeof(recv_buff) };
    if (twai_node_receive_from_isr(handle, &rx_frame) != ESP_OK) return false;

    CanFrame cf;
    cf.header = rx_frame.header;
    cf.len    = rx_frame.header.dlc;
    memcpy(cf.data, rx_frame.buffer, cf.len);

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(self->can_queue_, &cf, &woken);
    return woken == pdTRUE;
}

void ModuleCore::canProcessTask() {
    CanFrame frame;
    while (true) {
        if (!xQueueReceive(can_queue_, &frame, portMAX_DELAY)) continue;
        bool handled = handleCan(&frame);
        if (!handled && cfg_.on_can_rx) cfg_.on_can_rx(&frame);
    }
}

// ── CAN dispatcher ────────────────────────────────────────────────────────────

bool ModuleCore::handleCan(const CanFrame *frame) {
    if (frame->len == 0) return true;

    const uint8_t cmd = frame->data[0];

    if ((cmd == OTA_ACK || cmd == OTA_NAK) && bridge_ota_.active)  {
        bridgeHandleAck(frame);
        return true;
    }

    if (frame->header.id == can_id_) {
        if (cmd == OTA_DATA)  { targetHandleData(frame);  return true; }
        if (cmd == OTA_END)   { targetHandleEnd();         return true; }
        if (cmd == OTA_ABORT) { targetHandleAbort();       return true; }
    }

    if (frame->header.id == CAN_BROADCAST && cmd == CMD_DISCOVER) {
        vTaskDelay(pdMS_TO_TICKS(esp_random() % 50));
        uint8_t resp[4] = { MSG_DISCOVERY, can_id_, info_.module_type, info_.fw_version };
        sendCanFrame(can_id_, resp, 4);
        return true;
    }

    if (cmd == MSG_DISCOVERY && discovery_active_ && frame->len >= 4) {
        UartResponse resp{};
        resp.msg_type      = MSG_DISCOVERY;
        resp.source_device = frame->data[1];
        resp.data[0]       = frame->data[2];
        resp.data[1]       = frame->data[3];
        resp.data_len      = 2;
        sendUartResponse(resp);
        return true;
    }

    // ── Standard addressed commands ──
    if (frame->header.id == can_id_ || frame->header.id == CAN_BROADCAST) {
        switch (static_cast<Command>(cmd)) {
            case CMD_IDENTIFY:
                identify();
                return true;

            case CMD_SET_ID:
                if (frame->len < 2) return true;
                setId(frame->data[1]);
                return true;

            case CMD_GET_INFO: {
                uint8_t r[4] = { MSG_RESPONSE, info_.module_type, info_.fw_version, can_id_ };
                sendCanFrame(can_id_, r, 4);
                return true;
            }

            default: return false;
        }
    }

    return false;
}

// ── UART receive ──────────────────────────────────────────────────────────────

void ModuleCore::uartRxTaskEntry(void *arg) {
    static_cast<ModuleCore *>(arg)->uartRxLoop();
    vTaskDelete(nullptr);
}

void ModuleCore::uartRxLoop() {
    uint8_t buf[256];
    while (true) {
        int len = uart_read_bytes(uart_port_, buf, sizeof(buf), pdMS_TO_TICKS(50));
        if (len <= 0) continue;
        bool handled = handleUart(buf, static_cast<size_t>(len));
        if (!handled && cfg_.on_uart_rx) cfg_.on_uart_rx(buf, static_cast<size_t>(len));
    }
}

// ── UART dispatcher ───────────────────────────────────────────────────────────

esp_err_t ModuleCore::handleUart(const uint8_t *data, size_t len) {
    if (len < 2) return ESP_ERR_INVALID_SIZE;

    const uint8_t target  = data[0];
    const uint8_t command = data[1];
    const uint8_t *payload = data + 2;
    const size_t   plen    = len - 2;

    if (command == CMD_OTA_DATA) {
        if (plen < 3) return ESP_ERR_INVALID_SIZE;
        uint16_t seq  = ((uint16_t)payload[0] << 8) | payload[1];
        bool     last = payload[2] != 0;
        return bridgeSendChunk(target, seq, payload + 3, plen - 3, last);
    }

    if (command == CMD_OTA_ABORT) {
        return bridgeAbort();
    }

    // ── Standard commands ──
    if (target == can_id_ || target == 0x00) {
        switch (static_cast<Command>(command)) {
            case CMD_IDENTIFY:
                identify();
                return ESP_OK;

            case CMD_SET_ID:
                if (plen < 1) return ESP_ERR_INVALID_SIZE;
                return setId(payload[0]);

            case CMD_GET_INFO: {
                UartResponse resp{};
                resp.msg_type      = MSG_RESPONSE;
                resp.source_device = can_id_;
                resp.data[0]       = info_.module_type;
                resp.data[1]       = info_.fw_version;
                resp.data[2]       = can_id_;
                resp.data_len      = 3;
                return sendUartResponse(resp);
            }

            case CMD_DISCOVER:
                return startDiscovery();

            default:
                return ESP_ERR_NOT_SUPPORTED;
        }
    }

    uint32_t can_target = (target == 0xFF) ? CAN_BROADCAST : target;
    uint8_t  can_data[8];
    can_data[0] = command;
    if (plen > 0) memcpy(&can_data[1], payload, std::min(plen, (size_t)7));
    return sendCanFrame(can_target, can_data, plen + 1);
}

// ── UART response ─────────────────────────────────────────────────────────────

esp_err_t ModuleCore::sendUartResponse(const UartResponse &resp) {
    uint8_t buf[8];
    buf[0] = static_cast<uint8_t>(resp.msg_type);
    buf[1] = resp.source_device;
    memcpy(&buf[2], resp.data, resp.data_len);
    int n = uart_write_bytes(uart_port_, buf, 2 + resp.data_len);
    return (n == static_cast<int>(2 + resp.data_len)) ? ESP_OK : ESP_FAIL;
}

// ── CAN transmit ──────────────────────────────────────────────────────────────

esp_err_t ModuleCore::sendCanFrame(uint32_t can_id, const uint8_t *data, size_t len) {
    uint8_t tx_buf[8];
    memcpy(tx_buf, data, len);

    twai_frame_t frame = {};
    frame.header.id  = can_id;
    frame.header.ide = (can_id > 0x7FF) ? 1 : 0;
    frame.header.dlc = len;
    frame.buffer     = tx_buf;
    frame.buffer_len = sizeof(tx_buf);

    return twai_node_transmit(twai_hdl_, &frame, pdMS_TO_TICKS(100));
}

// ── Standard commands ─────────────────────────────────────────────────────────

esp_err_t ModuleCore::setId(uint8_t id) {
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open("storage", NVS_READWRITE, &handle), TAG, "NVS open failed");
    esp_err_t err = nvs_set_u8(handle, CAN_NVS_KEY, id);
    if (err == ESP_OK) { nvs_commit(handle); can_id_ = id; }
    nvs_close(handle);
    return err;
}

esp_err_t ModuleCore::startDiscovery() {
    if (discovery_active_) return ESP_ERR_INVALID_STATE;
    discovery_active_ = true;

    uint8_t uart_buf[4] = { MSG_DISCOVERY, can_id_, info_.module_type, info_.fw_version };
    uart_write_bytes(uart_port_, uart_buf, 4);

    uint8_t can_data[1] = { CMD_DISCOVER };
    sendCanFrame(CAN_BROADCAST, can_data, 1);

    spawnTask<&ModuleCore::discoveryTask>("discovery", 2048, 5);
    return ESP_OK;
}

void ModuleCore::identify() {
    spawnTask<&ModuleCore::identifyTask>("identify", 2048, 5);
}

void ModuleCore::identifyTask() {
    int original = gpio_get_level(info_.blink_pin);
    for (uint8_t i = 0; i < 10; i++) {
        gpio_set_level(info_.blink_pin, i % 2);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    gpio_set_level(info_.blink_pin, original);
}

void ModuleCore::discoveryTask() {
    vTaskDelay(pdMS_TO_TICKS(300));
    discovery_active_ = false;
}

uint8_t ModuleCore::loadId() {
    nvs_handle_t handle;
    if (nvs_open("storage", NVS_READONLY, &handle) != ESP_OK) return CAN_ID_UNSET;
    uint8_t id = CAN_ID_UNSET;
    nvs_get_u8(handle, CAN_NVS_KEY, &id);
    nvs_close(handle);
    return id;
}

void ModuleCore::appSupervisorTask() {
    for (;;) {
        auto res = cfg_.app_main();
        if (res.has_value()) {
            ESP_LOGI(TAG, "app_main exited OK, restarting");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        ESP_LOGE(TAG, "app_main failed (%s), restarting", res.error().get_message());
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

esp_err_t ModuleCore::bridgeSendChunk(uint8_t target, uint16_t seq,
                                       const uint8_t *data, size_t len, bool last) {
    if (len == 0 || len > 5) return ESP_ERR_INVALID_SIZE;

    if (!bridge_ota_.active) {
        bridge_ota_.target_id = target;
        bridge_ota_.next_seq  = 0;
        bridge_ota_.ack_sem   = xSemaphoreCreateBinary();
        if (!bridge_ota_.ack_sem) return ESP_ERR_NO_MEM;
        bridge_ota_.active = true;
        ESP_LOGI(TAG, "OTA bridge started → 0x%02X", target);
    }

    if (target != bridge_ota_.target_id) return ESP_ERR_INVALID_STATE;

    uint8_t frame[8];
    frame[0] = OTA_DATA;
    frame[1] = (seq >> 8) & 0xFF;
    frame[2] =  seq       & 0xFF;
    memcpy(&frame[3], data, len);

    for (uint8_t attempt = 0; attempt < cfg_.ota_max_retries; ++attempt) {
        bridge_ota_.next_seq = seq;
        bridge_ota_.ack_ok   = false;

        sendCanFrame(target, frame, 3 + len);

        if (xSemaphoreTake(bridge_ota_.ack_sem, pdMS_TO_TICKS(cfg_.ota_ack_timeout_ms)) == pdTRUE
            && bridge_ota_.ack_ok) {

            uint8_t status[3] = { MSG_OTA_STATUS, target, OTA_STATUS_PROGRESS };
            uart_write_bytes(uart_port_, status, 3);

            if (last) {
                // Send OTA_END and wait for OTA_DONE (target reboots after this)
                uint8_t end[1] = { OTA_END };
                sendCanFrame(target, end, 1);

                // Reuse ack_sem to wait for OTA_DONE
                bool done_ok = false;
                if (xSemaphoreTake(bridge_ota_.ack_sem,
                                   pdMS_TO_TICKS(10000)) == pdTRUE) {
                    done_ok = bridge_ota_.ack_ok;
                }

                uint8_t final_status[3] = {
                    MSG_OTA_STATUS, target,
                    static_cast<uint8_t>(done_ok ? OTA_STATUS_DONE_OK : OTA_STATUS_DONE_FAIL)
                };
                uart_write_bytes(uart_port_, final_status, 3);

                vSemaphoreDelete(bridge_ota_.ack_sem);
                bridge_ota_ = {};
                ESP_LOGI(TAG, "OTA bridge finished → 0x%02X (%s)",
                         target, done_ok ? "ok" : "fail");
            }
            return ESP_OK;
        }

        ESP_LOGW(TAG, "OTA NAK/timeout seq=%u attempt=%u", seq, attempt + 1);
    }

    // All retries exhausted — abort
    ESP_LOGE(TAG, "OTA chunk seq=%u failed, aborting", seq);
    bridgeAbort();
    return ESP_FAIL;
}

esp_err_t ModuleCore::bridgeAbort() {
    if (!bridge_ota_.active) return ESP_ERR_INVALID_STATE;

    uint8_t frame[1] = { OTA_ABORT };
    sendCanFrame(bridge_ota_.target_id, frame, 1);

    uint8_t status[3] = { MSG_OTA_STATUS, bridge_ota_.target_id, OTA_STATUS_ABORTED };
    uart_write_bytes(uart_port_, status, 3);

    if (bridge_ota_.ack_sem) {
        xSemaphoreGive(bridge_ota_.ack_sem);   // unblock if waiting
        vSemaphoreDelete(bridge_ota_.ack_sem);
    }
    bridge_ota_ = {};
    ESP_LOGW(TAG, "OTA bridge aborted");
    return ESP_OK;
}

void ModuleCore::bridgeHandleAck(const CanFrame *frame) {
    if (!bridge_ota_.active || frame->len < 3) return;

    uint16_t seq = ((uint16_t)frame->data[1] << 8) | frame->data[2];
    if (seq != bridge_ota_.next_seq) return;

    bridge_ota_.ack_ok = (frame->data[0] == OTA_ACK);
    xSemaphoreGive(bridge_ota_.ack_sem);
}

void ModuleCore::targetHandleData(const CanFrame *frame) {
    if (frame->len < 4) return;   // need at least seq(2) + 1 data byte

    uint16_t seq      = ((uint16_t)frame->data[1] << 8) | frame->data[2];
    const uint8_t *chunk = &frame->data[3];
    size_t chunk_len  = frame->len - 3;

    if (!target_ota_.active) {
        target_ota_.partition = esp_ota_get_next_update_partition(nullptr);
        if (!target_ota_.partition) {
            ESP_LOGE(TAG, "OTA: no update partition found");
            uint8_t nak[4] = { OTA_NAK, frame->data[1], frame->data[2], 0x01 };
            sendCanFrame(frame->header.id, nak, 4);  // reply to sender
            return;
        }

        esp_err_t err = esp_ota_begin(target_ota_.partition,
                                      OTA_SIZE_UNKNOWN, &target_ota_.handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
            uint8_t nak[4] = { OTA_NAK, frame->data[1], frame->data[2], 0x02 };
            sendCanFrame(frame->header.id, nak, 4);
            return;
        }

        target_ota_.active   = true;
        target_ota_.next_seq = 0;
        ESP_LOGI(TAG, "OTA target: receiving firmware");
    }

    // Sequence check — NAK duplicates/gaps
    if (seq != target_ota_.next_seq) {
        ESP_LOGW(TAG, "OTA seq mismatch: expected %u got %u", target_ota_.next_seq, seq);
        uint8_t nak[4] = { OTA_NAK, frame->data[1], frame->data[2], 0x03 };
        sendCanFrame(frame->header.id, nak, 4);
        return;
    }

    esp_err_t err = esp_ota_write(target_ota_.handle, chunk, chunk_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
        uint8_t nak[4] = { OTA_NAK, frame->data[1], frame->data[2], 0x04 };
        sendCanFrame(frame->header.id, nak, 4);
        targetHandleAbort();
        return;
    }

    target_ota_.next_seq++;

    uint8_t ack[3] = { OTA_ACK, frame->data[1], frame->data[2] };
    sendCanFrame(frame->header.id, ack, 3);
}

void ModuleCore::targetHandleEnd() {
    if (!target_ota_.active) return;

    esp_err_t err = esp_ota_end(target_ota_.handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        uint8_t done[2] = { OTA_DONE, 0x01 };
        sendCanFrame(CAN_BROADCAST, done, 2);   // sender may be any node
        target_ota_ = {};
        return;
    }

    err = esp_ota_set_boot_partition(target_ota_.partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        uint8_t done[2] = { OTA_DONE, 0x02 };
        sendCanFrame(CAN_BROADCAST, done, 2);
        target_ota_ = {};
        return;
    }

    ESP_LOGI(TAG, "OTA target: success, rebooting");
    uint8_t done[2] = { OTA_DONE, 0x00 };
    sendCanFrame(CAN_BROADCAST, done, 2);

    // Small delay to let the CAN frame transmit before restart
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

void ModuleCore::targetHandleAbort() {
    if (!target_ota_.active) return;
    esp_ota_abort(target_ota_.handle);
    target_ota_ = {};
    ESP_LOGW(TAG, "OTA target: aborted");
}
