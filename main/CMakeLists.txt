cmake_minimum_required(VERSION 3.16)

idf_component_register(
    SRCS
        "main.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        freertos
        esp_common
        nvs_flash
        esp_timer
        driver          # uart, gpio
        esp_adc         # adc_oneshot
        bt              # BLE Bluedroid (GATTS + GAP)
)

# Link math library (used by NMEA parser)
target_link_libraries(${COMPONENT_LIB} INTERFACE m)