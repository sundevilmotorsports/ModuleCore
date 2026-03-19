idf_component_register(SRCS "module_core.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES nvs_flash driver esp_driver_twai)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++23)
