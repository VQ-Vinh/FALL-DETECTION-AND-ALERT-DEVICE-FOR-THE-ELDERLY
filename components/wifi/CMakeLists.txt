# Component WiFi
#
# WiFi Station mode for ESP32
#
# Yêu cầu:
# - ESP-IDF framework
# - esp_wifi, esp_netif, esp_event

idf_component_register(SRCS "wifi.c"
                       INCLUDE_DIRS "." ".."
                       REQUIRES esp_wifi esp_netif esp_event lwip)