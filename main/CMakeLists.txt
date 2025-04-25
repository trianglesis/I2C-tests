idf_component_register(
                SRCS 
                "main.c"
                # "esp_bme680/bme680.c"
                # "esp_type_utils/type_utils.c"
                    
    INCLUDE_DIRS 
                "."
                # "./components/esp_bme680"
                # "./components/esp_bme680/include"
                # "./components/esp_type_utils"
                # "./components/esp_type_utils/include"
    )