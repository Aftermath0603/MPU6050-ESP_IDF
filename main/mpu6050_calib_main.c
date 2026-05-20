#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

#define TAG "MPU6050"

/* 硬件配置 */
#define I2C_MASTER_SDA_IO           GPIO_NUM_7
#define I2C_MASTER_SCL_IO           GPIO_NUM_8
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000      //I2C主机通信时钟频率，标准100Hz，快速400Hz
#define I2C_MASTER_TIMEOUT_MS       500         //通信超市时间：100-500ms

/* MPU6050 寄存器 */
#define MPU6050_ADDRESS             0x68
#define MPU6050_RA_PWR_MGMT_1       0x6B    //电源管理1号寄存器，核心控休眠、唤醒、时钟源选择
#define MPU6050_RA_ACCEL_XOUT_H     0x3B    //加速度计X轴输出高8位寄存器地址,后接温度、陀螺仪数据，可连续一次性读取14字节批量拿全数据
#define MPU6050_RA_TEMP_OUT_H       0x41    //MPU6050芯片内部温度数据高8位寄存器地址

// 三重校准偏移量
float accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0;
float gyro_x_offset  = 0, gyro_y_offset  = 0, gyro_z_offset  = 0;

// 温度补偿
float temp_ref = 0;
float ax_temp_bias = 0, ay_temp_bias = 0, az_temp_bias = 0;
float gx_temp_bias = 0, gy_temp_bias = 0, gz_temp_bias = 0;

// I2C 基础函数
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    esp_err_t err = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C驱动安装失败");
        return err;
    }
    ESP_LOGI(TAG, "I2C 初始化成功");
    return ESP_OK;
}

static esp_err_t mpu_write(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDRESS, buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

static esp_err_t mpu_read(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDRESS, &reg, 1, data, len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

// 读取温度
float mpu_read_temp(void) {
    uint8_t buf[2];
    mpu_read(MPU6050_RA_TEMP_OUT_H, buf, 2);
    int16_t raw_temp = (buf[0] << 8) | buf[1];
    return (raw_temp / 340.0f) + 36.53f;
}

// 上电调零
void mpu6050_calibrate(void)
{
    ESP_LOGW(TAG, "=====================================");
    ESP_LOGW(TAG, "  上电校准 → 请保持静止！");
    ESP_LOGW(TAG, "=====================================");
    vTaskDelay(1000);

    int32_t ax_sum=0,ay_sum=0,az_sum=0,gx_sum=0,gy_sum=0,gz_sum=0;
    const int total = 2000;
    for (int i = 0; i < total; i++) {
        uint8_t buf[14];
        mpu_read(MPU6050_RA_ACCEL_XOUT_H, buf,14);

        int16_t ax = (buf[0]<<8)|buf[1];
        int16_t ay = (buf[2]<<8)|buf[3];
        int16_t az = (buf[4]<<8)|buf[5];
        int16_t gx = (buf[8]<<8)|buf[9];
        int16_t gy = (buf[10]<<8)|buf[11];
        int16_t gz = (buf[12]<<8)|buf[13];

        ax_sum += ax;
        ay_sum += ay;
        az_sum += az - 16384;
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        if (i%200 == 0) {
            ESP_LOGI(TAG, "校准进度：%d%%", i*100/total);
        }
        vTaskDelay(1);
    }

    accel_x_offset = ax_sum / 2000.0f;
    accel_y_offset = ay_sum / 2000.0f;
    accel_z_offset = az_sum / 2000.0f;
    gyro_x_offset  = gx_sum / 2000.0f;
    gyro_y_offset  = gy_sum / 2000.0f;
    gyro_z_offset  = gz_sum / 2000.0f;

    temp_ref = mpu_read_temp();
    ESP_LOGI(TAG, "=====================================");
    ESP_LOGI(TAG, "  上电校准完成！ 温度：%.1f°C", temp_ref);
    ESP_LOGI(TAG, "ACC: %.1f, %.1f, %.1f", accel_x_offset, accel_y_offset, accel_z_offset);
    ESP_LOGI(TAG, "GYR: %.1f, %.1f, %.1f", gyro_x_offset, gyro_y_offset, gyro_z_offset);
    ESP_LOGI(TAG, "=====================================");
}

// 温度补偿计算
void update_temp_comp(void) {
    float new_temp = mpu_read_temp();
    float dt = new_temp - temp_ref;

    ax_temp_bias = dt * 2.0f;   //温漂系数，可自己标定
    ay_temp_bias = dt * 1.2f;
    az_temp_bias = dt * 0.8f;
    gx_temp_bias = dt * 0.6f;
    gy_temp_bias = dt * 0.6f;
    gz_temp_bias = dt * 0.6f;
}

// 读取补偿后的数据
void mpu6050_get_calibrated(float *out_ax, float *out_ay, float *out_az,
                            float *out_gx, float *out_gy, float *out_gz)
{
    uint8_t buf[14];
    mpu_read(MPU6050_RA_ACCEL_XOUT_H, buf,14);

    int16_t ax = (buf[0]<<8)|buf[1];
    int16_t ay = (buf[2]<<8)|buf[3];
    int16_t az = (buf[4]<<8)|buf[5];
    int16_t gx = (buf[8]<<8)|buf[9];
    int16_t gy = (buf[10]<<8)|buf[11];
    int16_t gz = (buf[12]<<8)|buf[13];

    // 同时减去：基础偏移 + 温度偏移
    *out_ax = ax - accel_x_offset - ax_temp_bias;
    *out_ay = ay - accel_y_offset - ay_temp_bias;
    *out_az = az - accel_z_offset - az_temp_bias;
    *out_gx = gx - gyro_x_offset  - gx_temp_bias;
    *out_gy = gy - gyro_y_offset  - gy_temp_bias;
    *out_gz = gz - gyro_z_offset  - gz_temp_bias;
}

// 静止状态下自动重新校准
void auto_recalibrate(void)
{
    ESP_LOGW(TAG, "==================================================");
    ESP_LOGW(TAG, "  🔴 触发自动重新调零");
    ESP_LOGW(TAG, "==================================================");

    int32_t ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
    for (int i=0; i<500; i++) {
        uint8_t buf[14];
        mpu_read(MPU6050_RA_ACCEL_XOUT_H, buf,14);
        ax += (int16_t)(buf[0]<<8|buf[1]);
        ay += (int16_t)(buf[2]<<8|buf[3]);
        az += (int16_t)(buf[4]<<8|buf[5]) - 16384;
        gx += (int16_t)(buf[8]<<8|buf[9]);
        gy += (int16_t)(buf[10]<<8|buf[11]);
        gz += (int16_t)(buf[12]<<8|buf[13]);
        vTaskDelay(1);
    }

    // 更新偏移
    accel_x_offset = ax/500.0f;
    accel_y_offset = ay/500.0f;
    accel_z_offset = az/500.0f;
    gyro_x_offset  = gx/500.0f;
    gyro_y_offset  = gy/500.0f;
    gyro_z_offset  = gz/500.0f;
    temp_ref = mpu_read_temp();

    // 输出最新偏移量
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "  🔵 重新调零完成！最新偏移量：");
    ESP_LOGI(TAG, "TEMP: %.1f", temp_ref);
    ESP_LOGI(TAG, "ACC: %.1f, %.1f, %.1f", accel_x_offset, accel_y_offset, accel_z_offset);
    ESP_LOGI(TAG, "GYR: %.1f, %.1f, %.1f", gyro_x_offset, gyro_y_offset, gyro_z_offset);
    ESP_LOGI(TAG, "==================================================");
}

// 主函数
void app_main(void)
{
    i2c_master_init();
    mpu_write(MPU6050_RA_PWR_MGMT_1, 0x00);
    vTaskDelay(100);
    ESP_LOGI(TAG, "MPU6050 初始化成功");

    // 1. 上电校准
    mpu6050_calibrate();

    // 2. 进入温漂自动维护模式
    ESP_LOGI(TAG, "=====================================");
    ESP_LOGI(TAG, "  ✅ 开始实时输出校准后数据");
    ESP_LOGI(TAG, "=====================================");

    TickType_t last_calib_tick = xTaskGetTickCount();
    const TickType_t calib_interval = pdMS_TO_TICKS(5000); //检查的时间间隔

    while (1) {
        // 读取补偿后数据
        float ax, ay, az, gx, gy, gz;
        mpu6050_get_calibrated(&ax, &ay, &az, &gx, &gy, &gz);

        // 调零后实时输出数据
        ESP_LOGI(TAG, "ACC: %6.0f | %6.0f | %6.0f", ax, ay, az);
        ESP_LOGI(TAG, "GYR: %6.0f | %6.0f | %6.0f", gx, gy, gz);
        ESP_LOGI(TAG, "==================================================");

        // 更新温度补偿
        update_temp_comp();

        // 5秒检查是否静止 → 自动重校准
        if (xTaskGetTickCount() - last_calib_tick >= calib_interval) {
            last_calib_tick = xTaskGetTickCount();

            // 读陀螺仪原始值判断静止
            uint8_t buf[14];
            mpu_read(MPU6050_RA_ACCEL_XOUT_H, buf,14);
            int16_t gx_raw = (buf[8]<<8)|buf[9];
            int16_t gy_raw = (buf[10]<<8)|buf[11];
            int16_t gz_raw = (buf[12]<<8)|buf[13];

            if (abs(gx_raw) < 15 && abs(gy_raw) <15 && abs(gz_raw) <15) {
                auto_recalibrate(); // 重调零
            }
        }
        vTaskDelay(200);
    }
}