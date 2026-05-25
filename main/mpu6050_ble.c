#include "mpu6050_ble.h"
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG_BRIDGE "UART_BRIDGE"

/* 串口配置 */
#define BRIDGE_UART_NUM      UART_NUM_1
#define BRIDGE_TX_IO         5
#define BRIDGE_RX_IO         4
#define BRIDGE_BAUD_RATE     115200
#define BUF_SIZE             1024

static bool bt_connected = false;

// 接收 C3 状态反馈的任务
static void uart_bridge_rx_task(void *arg)
{
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(BRIDGE_UART_NUM, data, BUF_SIZE, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0';
            if (strstr((char*)data, "[CONN]")) {
                bt_connected = true;
                ESP_LOGW(TAG_BRIDGE, "手机已连接 (通过 C3 桥接)");
            } else if (strstr((char*)data, "[DISC]")) {
                bt_connected = false;
                ESP_LOGW(TAG_BRIDGE, "手机已断开 (通过 C3 桥接)");
            }
        }
    }
    free(data);
}

void ble_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = BRIDGE_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_RTC,
    };

    ESP_ERROR_CHECK(uart_driver_install(BRIDGE_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BRIDGE_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(BRIDGE_UART_NUM, BRIDGE_TX_IO, BRIDGE_RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 启动状态监听任务
    xTaskCreate(uart_bridge_rx_task, "uart_bridge_rx", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG_BRIDGE, "ESP-P4 串口桥接初始化成功 (TX:%d, RX:%d)", BRIDGE_TX_IO, BRIDGE_RX_IO);
}

void ble_send_data(const char *data)
{
    if (data == NULL) return;
    uart_write_bytes(BRIDGE_UART_NUM, data, strlen(data));
}

bool ble_is_connected(void)
{
    return bt_connected;
}
