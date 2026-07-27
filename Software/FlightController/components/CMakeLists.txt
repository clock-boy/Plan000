# components/vl53l1x_uld/CMakeLists.txt
idf_component_register(
    SRCS "VL53L1X_api.c"
         "VL53L1X_calibration.c"
         "vl53l1_platform.c"
    INCLUDE_DIRS "."
    REQUIRES driver
)