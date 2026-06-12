idf_component_register(SRCS "main.c" "max30102.c" "dsp.c"
                    INCLUDE_DIRS "."
                    REQUIRES bt driver nvs_flash)