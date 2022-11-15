idf_component_register(SRCS "maintask.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver
                             mpu6050
                             ds18b20
                             ssd1306)
