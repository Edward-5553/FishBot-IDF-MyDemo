idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash BSP micro_ros_espidf_component
)
