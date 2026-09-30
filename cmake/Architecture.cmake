# Presentation may depend on value models and ports, never execution libraries.
function(onboard_autonomy_assert_model_dependencies target)
    set(model_targets
        onboard_autonomy_public_headers
        onboard_autonomy_mission_types
        onboard_autonomy_mission_camera_port
        onboard_autonomy_mission_target_detector_port
        onboard_autonomy_diagnostics_preview_port
    )
    set(pending ${target})
    set(visited)
    while(pending)
        list(POP_FRONT pending current)
        if(current IN_LIST visited)
            continue()
        endif()
        list(APPEND visited ${current})
        get_target_property(private_links ${current} LINK_LIBRARIES)
        get_target_property(public_links ${current} INTERFACE_LINK_LIBRARIES)
        # Extract project targets inside generator expressions as well.
        string(REGEX MATCHALL "onboard_autonomy_[A-Za-z0-9_]+"
            dependencies "${private_links};${public_links}")
        foreach(dependency IN LISTS dependencies)
            if(NOT dependency IN_LIST model_targets)
                message(FATAL_ERROR
                    "${target}: presentation depends on execution target ${dependency} via ${current}")
            endif()
            list(APPEND pending ${dependency})
        endforeach()
    endwhile()
endfunction()
