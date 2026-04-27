idf_component_register(SRCS "CODE.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver mpu6050 fall_detection wifi webserver esp_netif esp_http_server nvs_flash esp_wifi telegram)
