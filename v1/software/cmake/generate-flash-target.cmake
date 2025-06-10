#
# Defines the generate_flash_target() function.
#
# This function creates a custom target to flash an executable using OpenOCD.
# It intelligently resolves paths for configuration files. If a path is found
# relative to the project root, it uses the full path. Otherwise, it passes
# the path directly to OpenOCD, allowing the use of system-wide scripts
# (e.g., 'interface/stlink.cfg').
#
# Usage:
#   generate_flash_target(
#       TARGET <name_of_executable_target>  # REQUIRED: The executable target to flash.
#       INTERFACE <path_to_interface_cfg>   # REQUIRED: OpenOCD cfg for your debugger.
#                                           # Can be a project-local or system path.
#       MCU <path_to_mcu_cfg>               # REQUIRED: OpenOCD cfg for your MCU.
#                                           # Can be a project-local or system path.
#       EXTRA_ARGS <args...>                # OPTIONAL: Any extra arguments to pass to OpenOCD.
#   )
#

# Internal helper function to resolve OpenOCD config paths.
# It checks if a path exists relative to the project and, if so, makes it absolute.
# Otherwise, it returns the path as-is for OpenOCD's search mechanism.
function(_resolve_openocd_config_path USER_PATH OUT_VAR)
    set(FINAL_PATH ${USER_PATH}) # Default to the user-provided path

    if(NOT IS_ABSOLUTE "${USER_PATH}")
        set(PROJECT_RELATIVE_PATH "${CMAKE_SOURCE_DIR}/${USER_PATH}")
        if(EXISTS "${PROJECT_RELATIVE_PATH}")
            # The file exists within our project, so use the full path.
            set(FINAL_PATH "${PROJECT_RELATIVE_PATH}")
            message(DEBUG "generate_flash_target: Resolved local config file: ${FINAL_PATH}")
        else()
            # The file does NOT exist in our project. Assume it's a system path
            # that OpenOCD should find on its own (e.g., 'interface/stlink.cfg').
            # We don't modify the path in this case.
            message(DEBUG "generate_flash_target: Assuming OpenOCD system config: ${USER_PATH}")
        endif()
    endif()

    # Set the output variable in the parent scope.
    set(${OUT_VAR} "${FINAL_PATH}" PARENT_SCOPE)
endfunction()


function(generate_flash_target)
    set(options "")
    set(oneValueArgs "TARGET" "INTERFACE" "MCU")
    set(multiValueArgs "EXTRA_ARGS")
    cmake_parse_arguments(FLASH "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # --- Argument Validation ---
    if(NOT FLASH_TARGET)
        message(FATAL_ERROR "generate_flash_target: Missing required argument: TARGET")
    endif()
    if(NOT FLASH_INTERFACE)
        message(FATAL_ERROR "generate_flash_target: Missing required argument: INTERFACE")
    endif()
    if(NOT FLASH_MCU)
        message(FATAL_ERROR "generate_flash_target: Missing required argument: MCU")
    endif()
    if(NOT TARGET ${FLASH_TARGET})
        message(FATAL_ERROR "generate_flash_target: The specified TARGET '${FLASH_TARGET}' does not exist.")
    endif()

    # --- Find OpenOCD Executable ---
    find_program(OPENOCD_EXECUTABLE openocd)
    if(NOT OPENOCD_EXECUTABLE)
        message(WARNING "OpenOCD executable not found. The 'flash-${FLASH_TARGET}' target will not be available.")
        return()
    endif()

    # --- Resolve Paths for Config Files using our helper ---
    _resolve_openocd_config_path("${FLASH_INTERFACE}" INTERFACE_CFG_PATH)
    _resolve_openocd_config_path("${FLASH_MCU}" MCU_CFG_PATH)

    # --- Create the Custom Flash Target ---
    set(FLASH_CUSTOM_TARGET_NAME "flash-${FLASH_TARGET}")
    set(ELF_FILE_PATH "$<TARGET_FILE:${FLASH_TARGET}>")

    add_custom_target(${FLASH_CUSTOM_TARGET_NAME}
        COMMAND ${OPENOCD_EXECUTABLE}
            -f "${INTERFACE_CFG_PATH}"
            -f "${MCU_CFG_PATH}"
            ${FLASH_EXTRA_ARGS}
            -c "program ${ELF_FILE_PATH} verify reset exit"
        DEPENDS ${FLASH_TARGET}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Flashing ${FLASH_TARGET} with OpenOCD..."
        VERBATIM
    )

    message(STATUS "Created flash target: '${FLASH_CUSTOM_TARGET_NAME}' for executable '${FLASH_TARGET}'")

endfunction()
