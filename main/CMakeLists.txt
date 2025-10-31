idf_component_register(
    SRCS "main.c" "main_bak.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash BSP LIB micro_ros_espidf_component
)
