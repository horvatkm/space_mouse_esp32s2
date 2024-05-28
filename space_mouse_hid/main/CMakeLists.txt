idf_component_register(
    SRCS "sm_hid.cpp" "adcdata.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_adc esp_timer hal
    )
