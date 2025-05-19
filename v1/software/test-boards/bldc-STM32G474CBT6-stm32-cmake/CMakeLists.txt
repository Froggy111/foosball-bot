cmake_minimum_required(VERSION 4.0)
include(../../cmake/repo-common.cmake)

set(CMAKE_TOOLCHAIN_FILE "${STM32_CMAKE_PATH}/cmake/stm32_gcc.cmake")

project(bldc C CXX ASM)
stm32_fetch_cube(G4)

find_package(CMSIS COMPONENTS STM32G4 REQUIRED)
find_package(HAL COMPONENTS STM32G4 REQUIRED)
find_package(FreeRTOS COMPONENTS STM32G4 REQUIRED)
