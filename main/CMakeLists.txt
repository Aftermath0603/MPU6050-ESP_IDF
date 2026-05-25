idf_component_register(SRCS "mpu6050_ble.c" "mpu6050_filter.c" "mpu6050_calib_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver)
