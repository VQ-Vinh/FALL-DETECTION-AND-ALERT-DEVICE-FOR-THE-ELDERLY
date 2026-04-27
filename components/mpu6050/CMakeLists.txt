# Component MPU6050 cho ESP-IDF
#
# Đây là component driver cho cảm biến MPU6050:
# - mpu6050.c/h: Driver đọc dữ liệu từ cảm biến
# - roll_pitch.c/h: Tính toán góc nghiêng (roll/pitch)
#
# Yêu cầu:
# - ESP-IDF framework
# - I2C driver component

idf_component_register(SRCS "mpu6050.c"
                        "roll_pitch.c"
                       INCLUDE_DIRS "." ".."
                       REQUIRES driver esp_timer)
