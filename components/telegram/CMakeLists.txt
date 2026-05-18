# Component Telegram Bot
#
# Gửi thông báo qua Telegram Bot khi:
# - Hệ thống khởi động
# - Phát hiện té ngã
# - Nhấn nút SOS
#
# Yêu cầu:
# - ESP-IDF framework
# - esp_http_client (for HTTPS requests to Telegram API)

idf_component_register(SRCS "telegram.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_http_client mbedtls)