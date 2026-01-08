idf_component_register(
    SRCS "bus.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_spi esp_driver_i2c board 
)
