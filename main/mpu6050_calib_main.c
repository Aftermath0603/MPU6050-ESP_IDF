#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"
#include <math.h>
#include "mpu6050_filter.h"
#include "mpu6050_ble.h"

#define TAG "MPU6050"

/* 硬件配置 */
#define I2C_MASTER_SDA_IO           GPIO_NUM_7
#define I2C_MASTER_SCL_IO           GPIO_NUM_8
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000      //I2C主机通信时钟频率，标准100Hz，快速400Hz
#define I2C_MASTER_TIMEOUT_MS       500         //通信超市时间：100-500ms

/* MPU6050 寄存器 */
#define MPU6050_ADDRESS             0x68
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_TEMP_OUT_H       0x41

/* 工作参数 */
#define WAKEUP_INTERVAL_MS      2000    // 休眠期间唤醒检查间隔
#define MOTION_THRESHOLD        300.0f  // 降低阈值以更灵敏地捕捉微小运动
#define STABLE_TIME_THRESHOLD_MS 5000   // 长时间变化不大则休眠的时间阈值
#define SAMPLE_INTERVAL_MS      20      // 采样间隔
#define GYRO_AUTO_ZERO_COUNT    100     // 连续100次低运动量则触发一次微调零

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

// 休眠控制
void mpu6050_sleep(void) {
    mpu_write(MPU6050_RA_PWR_MGMT_1, 0x40); // 设置 SLEEP 位为 1
    ESP_LOGI(TAG, "传感器进入休眠模式");
}

void mpu6050_wake(void) {
    mpu_write(MPU6050_RA_PWR_MGMT_1, 0x00); // 设置 SLEEP 位为 0
    vTaskDelay(pdMS_TO_TICKS(100)); // 等待传感器稳定
    ESP_LOGI(TAG, "传感器已唤醒");
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
    ESP_LOGI(TAG, "TEMP: %.2f", temp_ref);
    ESP_LOGI(TAG, "ACC: %.2f, %.2f, %.2f", accel_x_offset, accel_y_offset, accel_z_offset);
    ESP_LOGI(TAG, "GYR: %.2f, %.2f, %.2f", gyro_x_offset, gyro_y_offset, gyro_z_offset);
    ESP_LOGI(TAG, "==================================================");
}

// 3轴PWM输出初始化:
void pwm_3axis_init(void)
{
    // 定时器配置（共用一个定时器，三路PWM同步）
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,  // 0~255
        .freq_hz = 10000,                       // 10kHz
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ch0 = {
        .gpio_num = 15, //ax
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    ledc_channel_config(&ch0);

    ledc_channel_config_t ch1 = {
        .gpio_num = 16, //ay
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    ledc_channel_config(&ch1);

    ledc_channel_config_t ch2 = {
        .gpio_num = 17, //az
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    ledc_channel_config(&ch2);
}

void accel_3axis_to_pwm(float ax, float ay, float az)
{
    // 把 -2g ~ +2g 映射到 0~255
    int32_t duty_x = (int32_t)((ax + 2.0f) * 127.0f);
    int32_t duty_y = (int32_t)((ay + 2.0f) * 127.0f);
    int32_t duty_z = (int32_t)((az + 2.0f) * 127.0f);

    // 限幅（防止超出 0~255）
    duty_x = (duty_x < 0) ? 0 : (duty_x > 255) ? 255 : duty_x;
    duty_y = (duty_y < 0) ? 0 : (duty_y > 255) ? 255 : duty_y;
    duty_z = (duty_z < 0) ? 0 : (duty_z > 255) ? 255 : duty_z;

    // 分别输出到3个GPIO
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_x);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty_y);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty_z);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

// 主函数
void app_main(void)
{
    i2c_master_init();
    mpu6050_wake();
    
    // 1. 初始化滤波和蓝牙
    filter_init();
    ble_init();

    // 2. 上电校准
    mpu6050_calibrate();

    ESP_LOGI(TAG, "=====================================");
    ESP_LOGI(TAG, "  ✅ 进入 Mahony 滤波与蓝牙传输模式");
    ESP_LOGI(TAG, "=====================================");

    float ax, ay, az, gx, gy, gz;
    float prev_ax = 0, prev_ay = 0, prev_az = 0;
    float roll, pitch, yaw;
    float dt = SAMPLE_INTERVAL_MS / 1000.0f;
    char ble_buf[128];
    
    TickType_t last_motion_tick = xTaskGetTickCount();
    TickType_t last_accel_output_tick = xTaskGetTickCount();
    bool is_sleeping = false;
    int stationary_counter = 0;

    pwm_3axis_init();

    while (1) {
        if (is_sleeping) {
            vTaskDelay(pdMS_TO_TICKS(WAKEUP_INTERVAL_MS));
            mpu6050_wake();
        }

        // 读取补偿后数据
        mpu6050_get_calibrated(&ax, &ay, &az, &gx, &gy, &gz);

        // 计算运动强度
        float delta_accel = sqrt(pow(ax - prev_ax, 2) + pow(ay - prev_ay, 2) + pow(az - prev_az, 2));
        float gyro_mag = sqrt(pow(gx, 2) + pow(gy, 2) + pow(gz, 2));
        
        prev_ax = ax; prev_ay = ay; prev_az = az;

        // 动态零偏补偿逻辑
        if (gyro_mag < 50.0f && delta_accel < 50.0f) {
            stationary_counter++;
            if (stationary_counter > GYRO_AUTO_ZERO_COUNT) {
                gyro_x_offset += gx * 0.05f;
                gyro_y_offset += gy * 0.05f;
                gyro_z_offset += gz * 0.05f;
                stationary_counter = 0;
            }
        } else {
            stationary_counter = 0;
        }

        if (delta_accel > MOTION_THRESHOLD || gyro_mag > MOTION_THRESHOLD) {
            last_motion_tick = xTaskGetTickCount();
            if (is_sleeping) {
                ESP_LOGW(TAG, "检测到运动，结束休眠！");
                is_sleeping = false;
            }
        }

        if (!is_sleeping) {
            // 陀螺仪原始数据转换为弧度/秒
            float gx_rad = (gx / 131.0f) * M_PI / 180.0f;
            float gy_rad = (gy / 131.0f) * M_PI / 180.0f;
            float gz_rad = (gz / 131.0f) * M_PI / 180.0f;

            // 使用 Mahony 滤波进行姿态解算
            mahony_update(ax, ay, az, gx_rad, gy_rad, gz_rad, dt);
            get_euler_angles(&roll, &pitch, &yaw);

            // 每隔1秒输出一次加速度和姿态角，并发送到蓝牙
            if (xTaskGetTickCount() - last_accel_output_tick >= pdMS_TO_TICKS(1000)) {
                last_accel_output_tick = xTaskGetTickCount();
                
                // 格式化输出字符串
                snprintf(ble_buf, sizeof(ble_buf), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
                         ax, ay, az/16384.0, roll, pitch, yaw);
                
                // 串口输出
                //ESP_LOGI(TAG, "--- 实时输出 (1s) ---");
                printf("%s", ble_buf);
                //PWM输出
                accel_3axis_to_pwm(ax, ay, az/16384.0);
                // 蓝牙输出
                if (ble_is_connected()) {
                    ble_send_data(ble_buf);
                }
            }

            // 更新温度补偿
            update_temp_comp();

            // 检查是否长时间静止
            if (xTaskGetTickCount() - last_motion_tick > pdMS_TO_TICKS(STABLE_TIME_THRESHOLD_MS)) {
                ESP_LOGW(TAG, "长时间静止，准备进入休眠...");
                mpu6050_sleep();
                is_sleeping = true;
            }

            vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
        } else {
            // 如果唤醒检查后依然没有运动，继续休眠
            if (xTaskGetTickCount() - last_motion_tick > pdMS_TO_TICKS(STABLE_TIME_THRESHOLD_MS)) {
                mpu6050_sleep();
            }
        }
    }
}
