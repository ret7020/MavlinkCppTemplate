set(COMPILER $ENV{COMPILER})

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_C_COMPILER $ENV{COMPILER}/arm-rockchip830-linux-uclibcgnueabihf-gcc)
set(CMAKE_CXX_COMPILER $ENV{COMPILER}/arm-rockchip830-linux-uclibcgnueabihf-g++)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

