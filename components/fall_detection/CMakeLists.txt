# Component Fall Detection
#
# Thuat toan phat hien te nga cho nguoi cao tuoi
# Su dung du lieu tu MPU6050 (accel + gyro)
#
# Yêu cầu:
# - ESP-IDF framework
# - FreeRTOS

idf_component_register(SRCS "fall_detection.c"
                       INCLUDE_DIRS "." ".."
                       REQUIRES driver freertos)