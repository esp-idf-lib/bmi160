if (IDF_VERSION_MAJOR EQUAL 5 AND IDF_VERSION_MINOR GREATER_EQUAL 3)
    set(ESP_DRIVER_GPIO esp_driver_gpio)
endif()

idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES ${ESP_DRIVER_GPIO}
                    REQUIRES bmi160 )
