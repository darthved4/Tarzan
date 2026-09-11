if(CONFIG_Tarzan)
    set(Tarzan_DIR ${ZEPHYR_CURRENT_MODULE_DIR})
    zephyr_include_directories(${Tarzan_DIR}/include)
    zephyr_library()
    zephyr_library_sources(
        ${Tarzan_DIR}/lib/drive.c
        ${Tarzan_DIR}/lib/arm.c
        ${Tarzan_DIR}/lib/arm_pose.c
        ${Tarzan_DIR}/lib/cobs.c
        ${Tarzan_DIR}/lib/link_orientation.c
        ${Tarzan_DIR}/lib/madgwick.c
        ${Tarzan_DIR}/lib/orientation.c
        ${Tarzan_DIR}/lib/pid.c
    )
endif()
