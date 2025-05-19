if(REPO_COMMON_CMAKE_INCLUDED)
    return()
endif()
set(REPO_COMMON_CMAKE_INCLUDED TRUE)

message(STATUS "repo-common.cmake: Applying shared configurations...")

set(SOFTWARE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../"
    CACHE PATH "root directory of software directory" FORCE)

# stm32-cmake
set(STM32_CMAKE_PATH "${SOFTWARE_ROOT}/toolchains/stm32-cmake"
    CACHE PATH "path for root directory of stm32-cmake toolchain" FORCE)
message(STATUS "repo-common.cmake: STM32_CMAKE_PATH is ${STM32_CMAKE_PATH}")
set(STM32_CMAKE_MODULES_PATH "${STM32_CMAKE_PATH}/cmake"
    CACHE PATH "path for cmake modules of stm32-cmake toolchain" FORCE)
message(STATUS "repo-common.cmake: STM32_CMAKE_MODULES_PATH is ${STM32_CMAKE_MODULES_PATH}")
get_filename_component(STM32_CMAKE_MODULES_PATH
    "${STM32_CMAKE_MODULES_PATH}" ABSOLUTE)
message(STATUS "repo-common.cmake: STM32_CMAKE_MODULES_PATH is ${STM32_CMAKE_MODULES_PATH}")
message(STATUS "repo-common.cmake: Adding ${STM32_CMAKE_MODULES_PATH} to CMAKE_MODULE_PATH")
list(PREPEND CMAKE_MODULE_PATH "${STM32_CMAKE_MODULES_PATH}")
message(STATUS "repo-common.cmake: CMAKE_MODULE_PATH is ${CMAKE_MODULE_PATH}")

# make sure to generate compile_commands.jso
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

message(STATUS "repo-common.cmake: Shared configurations applied.")
