#ifndef MPU6050_BLE_H
#define MPU6050_BLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief 初始化与 ESP32-C3 桥接的 UART 串口
 * 串口配置：115200, TX=GPIO5, RX=GPIO4
 */
void ble_init(void);

/**
 * @brief 通过串口将数据发送给 ESP32-C3 桥接模块
 * @param data 要发送的字符串数据
 */
void ble_send_data(const char *data);

/**
 * @brief 检查桥接链路是否正常 (串口模式下默认返回 true)
 */
bool ble_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_BLE_H
