set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(TOOLCHAIN_DIR /opt/nxp-real-time-edge/2.5)
set(CMAKE_C_COMPILER ${TOOLCHAIN_DIR}/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_DIR}/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-g++)

set(CMAKE_SYSROOT ${TOOLCHAIN_DIR}/sysroots/armv8a-poky-linux)
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)