idf_component_register(
    SRCS
        "main.c"
        "motor_driver.c"
        "encoder.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        freertos
        esp_common
        log
)
