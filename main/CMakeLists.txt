# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(SRCS "MAXheartRate.c"  "heartrate.c" "temperature.c" "i2c.c" "MLX90614_API.c" "MLX90614_SMBus_Driver.c" "reading.c"  "scan.c"
                    INCLUDE_DIRS ".")