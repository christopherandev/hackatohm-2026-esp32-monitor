idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_adc
        esp_timer
        esp_rom
        lwip
        esp_driver_gptimer
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_spi
        esp_driver_sdspi
        fatfs
        sdmmc
        nvs_flash
        esp_wifi
        esp_event
        esp_netif
        mqtt
)
