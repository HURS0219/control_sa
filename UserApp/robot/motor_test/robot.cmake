# robot.cmake — GM6020 单电机安全上电测试 (motor_test)
# 目标板: GIMBAL_BOARD (STM32F407xx)

if (NOT DEFINED BOARD_TYPE)
    set(BOARD_TYPE "GIMBAL_BOARD")
endif ()

if (BOARD_TYPE STREQUAL "ONE_BOARD")
    set(MCU_TYPE "stm32-h7")
elseif (BOARD_TYPE STREQUAL "GIMBAL_BOARD")
    set(MCU_TYPE "stm32-f4")
elseif (BOARD_TYPE STREQUAL "CHASSIS_BOARD")
    set(MCU_TYPE "stm32-h7")
else ()
    message(FATAL_ERROR "Unknown BOARD_TYPE '${BOARD_TYPE}'")
endif ()

add_compile_definitions(${BOARD_TYPE})
add_compile_definitions(USE_RC_CTRL)

include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})

file(GLOB_RECURSE ROBOT_SOURCES
        CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
)

list(APPEND SOURCES ${ROBOT_SOURCES})
