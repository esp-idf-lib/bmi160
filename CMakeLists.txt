list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/eil-cmake-utils/cmake)
include(eil_check_idf_features)
eil_check_idf_has_esp_drivers()

set(REQUIRED_COMPONENTS freertos esp_idf_lib_helpers)
if(EIL_IDF_HAS_ESP_DRIVERS)
    list(APPEND REQUIRED_COMPONENTS esp_driver_gpio)
else()
    list(APPEND REQUIRED_COMPONENTS driver)
endif()

idf_component_register(SRCS "bmi160.c"
                       INCLUDE_DIRS "include"
                       REQUIRES ${REQUIRED_COMPONENTS})

include(eil_ci)
