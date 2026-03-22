#include "module_core.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_ota_ops.h"
#include "nvs.h"
#include <cstring>

static const char *TAG = "ModuleCore";

ModuleCore::~ModuleCore() {
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

    ESP_RETURN_ON_ERROR(uart_param_config(uart_port_, &uart_cfg), TAG, "UART param config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(uart_port_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "UART set pin failed");
    ESP_RETURN_ON_ERROR(uart_driver_install(uart_port_, 512, 512, 0, nullptr, 0), TAG, "UART install failed");

    can_queue_ = xQueueCreate(16, sizeof(CanFrame));

    twai_onchip_node_config_t node_cfg = {};
    node_cfg.io_cfg.tx             = cfg.can_tx;
    node_cfg.io_cfg.rx             = cfg.can_rx;
    node_cfg.io_cfg.quanta_clk_out    = GPIO_NUM_NC;
    node_cfg.io_cfg.bus_off_indicator = GPIO_NUM_NC;
    node_cfg.bit_timing            = cfg.can_timing;
    node_cfg.fail_retry_cnt        = 3;
    node_cfg.tx_queue_depth        = 16;

    ESP_RETURN_ON_ERROR(twai_new_node_onchip(&node_cfg, &twai_hdl_), TAG, "TWAI init failed");

    twai_event_callbacks_t cbs = {};
    cbs.on_rx_done = onCanRxStatic;
    ESP_RETURN_ON_ERROR(twai_node_register_event_callbacks(twai_hdl_, &cbs, this), TAG, "TWAI cb register failed");
    ESP_RETURN_ON_ERROR(twai_node_enable(twai_hdl_), TAG, "TWAI enable failed");

    spawnTask<&ModuleCore::canProcessTask>("can_proc", 4096, 6);
    xTaskCreate(uartRxTaskEntry, "uart_rx", 4096, this, 5, nullptr);

    ESP_LOGI(TAG, "Ready — CAN ID: 0x%02X", can_id_);

    if (cfg_.app_main) {
        spawnTask<&ModuleCore::appSupervisorTask>("app_sup", 8192, 5);
    }

    return ESP_OK;
}

esp_err_t ModuleCore::otaBegin(uint32_t total_size) {
    if (ota_.active) otaEnd(false);

    const esp_partition_t *part = esp_ota_get_next_update_partition(nullptr);
    if (!part) {
        uint8_t nack[4] = { MSG_OTA_ACK, can_id_, 0xFF, 0xFF };
        uart_write_bytes(uart_port_, nack, sizeof(nack));
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t err = esp_ota_begin(part, total_size, &ota_.handle);
    if (err != ESP_OK) {
        uint8_t nack[4] = { MSG_OTA_ACK, can_id_, 0xFF, 0xFF };
        uart_write_bytes(uart_port_, nack, sizeof(nack));
        return err;
    }

    ota_.active   = true;
    ota_.expected = total_size;
    ota_.written  = 0;

    esp_log_level_set("*", ESP_LOG_NONE);

    uint8_t ack[4] = { MSG_OTA_ACK, can_id_, 0xFE, 0xFF };
    uart_write_bytes(uart_port_, ack, sizeof(ack));
    return ESP_OK;
}

esp_err_t ModuleCore::otaWrite(const uint8_t *data, size_t len, uint16_t seq, uint8_t src_id) {
    if (!ota_.active) return ESP_ERR_INVALID_STATE;

    esp_err_t err = esp_ota_write(ota_.handle, data, len);
    if (err != ESP_OK) {
        otaEnd(false);
        return err;
    }
    ota_.written += len;

    uint8_t ack[4] = { MSG_OTA_ACK, src_id, (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8) };
    uart_write_bytes(uart_port_, ack, sizeof(ack));
    return ESP_OK;
}

esp_err_t ModuleCore::otaEnd(bool commit) {
    if (!ota_.active) return ESP_ERR_INVALID_STATE;

    esp_log_level_set("*", ESP_LOG_INFO);

    esp_err_t err = ESP_OK;
    if (commit) {
        err = esp_ota_end(ota_.handle);
        if (err == ESP_OK) {
            const esp_partition_t *part = esp_ota_get_next_update_partition(nullptr);
            err = esp_ota_set_boot_partition(part);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "OTA complete — rebooting");
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_restart();
            }
        }
        if (err != ESP_OK) ESP_LOGE(TAG, "OTA commit failed: %s", esp_err_to_name(err));
    } else {
        esp_ota_abort(ota_.handle);
    }

    ota_ = {};
    return err;
}

bool ModuleCore::onCanRxStatic(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *ctx) {
    auto *self = static_cast<ModuleCore*>(ctx);

    uint8_t recv_buff[8];
    twai_frame_t rx_frame = {};
    rx_frame.buffer = recv_buff;
    rx_frame.buffer_len = sizeof(recv_buff);

    if (twai_node_receive_from_isr(handle, &rx_frame) != ESP_OK) return false;

    CanFrame cf{};
    cf.header = rx_frame.header;
    cf.len    = rx_frame.header.dlc;
    memcpy(cf.data, rx_frame.buffer, cf.len);

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(self->can_queue_, &cf, &woken);
    return woken == pdTRUE;
}

void ModuleCore::canProcessTask() {
    CanFrame frame{};
    while (true) {
        if (!xQueueReceive(can_queue_, &frame, portMAX_DELAY)) continue;
        bool handled = handleCan(&frame);
        if (!handled && cfg_.on_can_rx) cfg_.on_can_rx(&frame);
    }
}

bool ModuleCore::handleCan(const CanFrame *frame) {
    if (frame->header.ide == 0) return false;

    ParsedArbId parsed = parse_arb_id(frame->header.id);
    uint8_t cmd = parsed.cmd;
    uint8_t target = parsed.target;
    const uint8_t *payload = frame->data;
    size_t payload_len = frame->len;

    if (cmd == CMD_DISCOVER && target == 0xFF) {
        vTaskDelay(pdMS_TO_TICKS(esp_random() % 50));
        uint8_t resp[3] = { can_id_, info_.module_type, info_.fw_version };
        sendCanFrame(build_arb_id(0x00, MSG_DISCOVERY), resp, 3);
        return true;
    }

    if (cmd == MSG_DISCOVERY && discovery_active_ && payload_len >= 3) {
        UartResponse resp{};
        resp.msg_type      = MSG_DISCOVERY;
        resp.source_device = payload[0];
        resp.data[0]       = payload[1];
        resp.data[1]       = payload[2];
        resp.data_len      = 2;
        sendUartResponse(resp);
        return true;
    }

    if (target == can_id_ || target == 0xFF) {
        switch (static_cast<Command>(cmd)) {
            case CMD_IDENTIFY:
                identify();
                return true;
            case CMD_SET_ID:
                if (payload_len >= 1) setId(payload[0]);
                return true;
            case CMD_GET_INFO: {
                uint8_t resp_payload[3] = { info_.module_type, info_.fw_version, can_id_ };
                sendCanFrame(build_arb_id(0x00, MSG_RESPONSE), resp_payload, 3);
                return true;
            }
            case CMD_OTA_BEGIN:
                if (payload_len >= 4) {
                    uint32_t sz = (uint32_t)payload[0]        | ((uint32_t)payload[1] << 8)
                                | ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);
                    otaBegin(sz);
                }
                return true;
            case CMD_OTA_DATA:
                if (payload_len >= 2) {
                    uint16_t seq = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
                    otaWrite(&payload[2], payload_len - 2, seq, 0x00);
                }
                return true;
            case CMD_OTA_END:
                otaEnd(payload_len >= 1 && payload[0] == 0x00);
                return true;
            case CMD_DATA:
                return false;
            default:
                return false;
        }
    }

    return false;
}

void ModuleCore::uartRxTaskEntry(void *arg) {
    static_cast<ModuleCore*>(arg)->uartRxLoop();
    vTaskDelete(nullptr);
}

void ModuleCore::uartRxLoop() {
    uint8_t buf[256];
    while (true) {
        int len = uart_read_bytes(uart_port_, buf, sizeof(buf), pdMS_TO_TICKS(50));
        if (len <= 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        esp_err_t err = handleUart(buf, static_cast<size_t>(len));
        if (err != ESP_OK && cfg_.on_uart_rx)
            cfg_.on_uart_rx(buf, static_cast<size_t>(len));
    }
}

esp_err_t ModuleCore::handleUart(const uint8_t *data, size_t len) {
    if (len < 2) return ESP_ERR_INVALID_SIZE;

    uint8_t         target      = data[0];
    uint8_t         command     = data[1];
    const uint8_t  *payload     = data + 2;
    size_t          payload_len = (len > 2) ? len - 2 : 0;

    if (target == 0x00 || target == can_id_) {
        if (static_cast<Command>(command) == CMD_DISCOVER)
            return startDiscovery();
    }

    if (target == can_id_) {
        switch (static_cast<Command>(command)) {
            case CMD_IDENTIFY:
                identify();
                return ESP_OK;

            case CMD_SET_ID:
                if (payload_len < 1) return ESP_ERR_INVALID_SIZE;
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

            case CMD_OTA_BEGIN: {
                if (payload_len < 4) return ESP_ERR_INVALID_SIZE;
                uint32_t sz = (uint32_t)payload[0]        | ((uint32_t)payload[1] << 8)
                            | ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);
                return otaBegin(sz);
            }

            case CMD_OTA_DATA: {
                if (payload_len < 3) return ESP_ERR_INVALID_SIZE;
                uint16_t seq = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
                return otaWrite(payload + 2, payload_len - 2, seq, can_id_);
            }

            case CMD_OTA_END:
                return otaEnd(payload_len >= 1 && payload[0] == 0x00);

            default:
                return ESP_ERR_NOT_SUPPORTED;
        }
    }

    switch (static_cast<Command>(command)) {
        case CMD_OTA_BEGIN: {
            uint8_t can_data[4] = {
                (uint8_t)(payload_len > 0 ? payload[0] : 0),
                (uint8_t)(payload_len > 1 ? payload[1] : 0),
                (uint8_t)(payload_len > 2 ? payload[2] : 0),
                (uint8_t)(payload_len > 3 ? payload[3] : 0)
            };
            uint8_t dest = (target == 0xFF) ? 0xFF : static_cast<uint8_t>(target);
            return sendCanFrame(build_arb_id(dest, CMD_OTA_BEGIN), can_data, 4);
        }
        case CMD_OTA_DATA: {
            if (payload_len < 2) return ESP_ERR_INVALID_SIZE;
            uint16_t seq          = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
            const uint8_t *fwdata = payload + 2;
            size_t fwlen          = payload_len - 2;
            for (size_t offset = 0; offset < fwlen; offset += OTA_CAN_CHUNK) {
                size_t chunk = std::min(fwlen - offset, (size_t)OTA_CAN_CHUNK);
                uint8_t can_data[8];
                can_data[0] = (uint8_t)(seq & 0xFF);
                can_data[1] = (uint8_t)(seq >> 8);
                memcpy(&can_data[2], fwdata + offset, chunk);
                uint8_t dest = (target == 0xFF) ? 0xFF : static_cast<uint8_t>(target);
                esp_err_t err = sendCanFrame(build_arb_id(dest, CMD_OTA_DATA), can_data, 2 + chunk);
                if (err != ESP_OK) return err;
                seq++;
            }
            return ESP_OK;
        }
        case CMD_OTA_END: {
            uint8_t can_data[1];
            can_data[0] = payload_len >= 1 ? payload[0] : 0x00;
            uint8_t dest = (target == 0xFF) ? 0xFF : static_cast<uint8_t>(target);
            return sendCanFrame(build_arb_id(dest, CMD_OTA_END), can_data, 1);
        }
        default: {
            uint8_t can_data[8];
            memcpy(can_data, payload, payload_len);
            uint8_t dest = (target == 0xFF) ? 0xFF : static_cast<uint8_t>(target);
            return sendCanFrame(build_arb_id(dest, static_cast<uint8_t>(command)), can_data, payload_len);
        }
    }
}

esp_err_t ModuleCore::sendUartResponse(const UartResponse &resp) {
    uint8_t buffer[8];
    buffer[0] = static_cast<uint8_t>(resp.msg_type);
    buffer[1] = resp.source_device;
    memcpy(&buffer[2], resp.data, resp.data_len);
    int written = uart_write_bytes(uart_port_, buffer, 2 + resp.data_len);
    return (written == static_cast<int>(2 + resp.data_len)) ? ESP_OK : ESP_FAIL;
}

esp_err_t ModuleCore::sendCanFrame(uint32_t can_id, const uint8_t *data, size_t len) {
    uint8_t tx_buff[8];
    if (len > sizeof(tx_buff)) len = sizeof(tx_buff);
    if (data != nullptr && len > 0) {
        memcpy(tx_buff, data, len);
    }

    twai_frame_t frame = {};
    frame.header.id  = can_id;
    frame.header.ide = 1;
    frame.header.dlc = len;
    frame.buffer     = tx_buff;
    frame.buffer_len = sizeof(tx_buff);

    return twai_node_transmit(twai_hdl_, &frame, pdMS_TO_TICKS(100));
}

esp_err_t ModuleCore::transmitCan(uint32_t can_id, const uint8_t *data, size_t len) {
    uint8_t tx_buff[8];
    tx_buff[0] = CMD_DATA;
    if (len > sizeof(tx_buff) - 1) len = sizeof(tx_buff) - 1;
    if (data != nullptr && len > 0) memcpy(&tx_buff[1], data, len);

    twai_frame_t frame = {};
    frame.header.id  = can_id;
    frame.header.ide = 1;
    frame.header.dlc = static_cast<uint8_t>(len + 1);
    frame.buffer     = tx_buff;
    frame.buffer_len = sizeof(tx_buff);

    return twai_node_transmit(twai_hdl_, &frame, pdMS_TO_TICKS(100));
}

esp_err_t ModuleCore::setId(uint8_t id) {
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open("storage", NVS_READWRITE, &handle), TAG, "NVS open failed");
    esp_err_t err = nvs_set_u8(handle, CAN_NVS_KEY, id);
    if (err == ESP_OK) {
        nvs_commit(handle);
        can_id_ = id;
    }
    nvs_close(handle);
    return err;
}

esp_err_t ModuleCore::startDiscovery() {
    discovery_active_ = true;
    uint8_t uart_buf[4] = { MSG_DISCOVERY, can_id_, info_.module_type, info_.fw_version };
    uart_write_bytes(uart_port_, uart_buf, 4);
    sendCanFrame(build_arb_id(0xFF, CMD_DISCOVER), nullptr, 0);
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
        if (!res.has_value()) {
            ESP_LOGE(TAG, "app_main failed: %s", res.error().get_message().data());
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    }
}
