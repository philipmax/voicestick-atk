idf_component_register(
    SRCS "stick_s3_board.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_i2c esp_adc
)
