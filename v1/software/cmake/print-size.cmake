# =============================================================================
# FUNCTION: target_print_size
#
# Adds a post-build command to a target to display its size using the
# platform's `size` utility.
#
# @param TARGET_NAME The name of the target (e.g., my_app, my_lib).
# =============================================================================
function(target_print_size TARGET_NAME)
    # 1. First, check if the CMAKE_SIZE utility is available on this system.
    #    If not, print a warning and do nothing.
    if(NOT CMAKE_SIZE)
        message(WARNING "The 'size' utility (CMAKE_SIZE) was not found. "
                        "Cannot add size check for target '${TARGET_NAME}'.")
        return() # Exit the function
    endif()

    # 2. Check if the target provided by the user actually exists.
    #    This prevents configuration errors.
    if(NOT TARGET ${TARGET_NAME})
        message(FATAL_ERROR "Target '${TARGET_NAME}' does not exist. "
                            "Cannot add post-build size check.")
        return() # Exit the function
    endif()

    # 3. Add the custom command to the specified target.
    add_custom_command(
        TARGET ${TARGET_NAME}
        POST_BUILD
        COMMAND
            ${CMAKE_SIZE} $<TARGET_FILE:${TARGET_NAME}>
        COMMENT
            "Displaying size of ${TARGET_NAME}..."
        VERBATIM
    )
endfunction()
