cmake_minimum_required(VERSION 3.10)
project(BME280_Driver VERSION 1.0.0 LANGUAGES CXX)

# Use C++17 for better standard library support (like std::byte or std::span)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# 1. Driver Library
# -----------------------------------------------------------------------------
# This creates a header-only or static library for the driver itself.
add_library(bme280_driver INTERFACE)

# Define the include directories for the driver
target_include_directories(bme280_driver INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# -----------------------------------------------------------------------------
# 2. Executable (Linux Implementation)
# -----------------------------------------------------------------------------
# This compiles the Linux implementation and the main entry point.
add_executable(bme280_example 
    src/main.cpp 
    src/LinuxI2CBus.cpp
    src/LinuxSPIBus.cpp
)

# Link the executable to the driver interface
target_link_libraries(bme280_example PRIVATE bme280_driver)

# -----------------------------------------------------------------------------
# 3. Compiler Warnings (The "Senior" touch)
# -----------------------------------------------------------------------------
# Enabling high warning levels shows you care about code quality.
if(MSVC)
    target_compile_options(bme280_example PRIVATE /W4 /WX)
else()
    target_compile_options(bme280_example PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
