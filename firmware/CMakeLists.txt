cmake_minimum_required(VERSION 3.16)
project(motor_firmware C)

set(CMAKE_C_STANDARD 11)
add_compile_options(-Wall -Wextra)

# Download the FreeRTOS kernel automatically (fixed version)
include(FetchContent)
FetchContent_Declare(freertos_kernel
  GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
  GIT_TAG        V11.1.0
  GIT_SHALLOW    TRUE)

# Tell FreeRTOS where our FreeRTOSConfig.h is
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/config)

# Use the POSIX port: FreeRTOS runs as a normal Linux program
set(FREERTOS_PORT GCC_POSIX CACHE STRING "" FORCE)
set(FREERTOS_HEAP 4 CACHE STRING "" FORCE)
FetchContent_MakeAvailable(freertos_kernel)

add_executable(motor_firmware src/main.c)
target_link_libraries(motor_firmware PRIVATE freertos_kernel freertos_config pthread)