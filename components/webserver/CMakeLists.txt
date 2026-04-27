# Component Webserver
#
# HTTP server for real-time sensor data display
#
# Yêu cầu:
# - ESP-IDF framework
# - esp_http_server

idf_component_register(SRCS "webserver.c"
                       INCLUDE_DIRS "." ".."
                       REQUIRES esp_http_server)