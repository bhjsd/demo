cmake_minimum_required(VERSION 3.22)

# This file is generated only once and then user-maintained.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME demo)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C ASM)
add_executable(${CMAKE_PROJECT_NAME})

# CubeMX generated sources/includes/libs
add_subdirectory(cmake/stm32cubemx)

# User include directories
set(USER_INCLUDE_DIRS
    Middlewares/Third_Party/SEGGER/RTT
    Middlewares/Third_Party/SEGGER/Config
    Drivers/CMSIS/DSP/Include
    bsp
    bsp/usart
    bsp/log
    bsp/can
    bsp/dwt
    bsp/gpio
    bsp/pwm
    bsp/spi
    bsp/usb
    modules
    modules/daemon
    modules/algorithm
    modules/message_center
    modules/master_machine
    modules/BMI088
    modules/imu
    modules/vofa
    modules/motor
    modules/motor/DJImotor
    modules/motor/DMmotor
    application
    application/cmd
    application/gimbal
    application/laser_ctrl
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${USER_INCLUDE_DIRS})

# User sources
set(USER_SOURCES
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT.c
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_printf.c
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s

    bsp/usart/bsp_usart.c
    bsp/log/bsp_log.c
    bsp/can/bsp_can.c
    bsp/dwt/bsp_dwt.c
    bsp/gpio/bsp_gpio.c
    bsp/pwm/bsp_pwm.c
    bsp/spi/bsp_spi.c
    bsp/usb/bsp_usb.c

    modules/algorithm/controller.c
    modules/algorithm/kalman_filter.c
    modules/algorithm/QuaternionEKF.c
    modules/algorithm/crc8.c
    modules/algorithm/crc16.c
    modules/algorithm/user_lib.c
    modules/message_center/message_center.c
    modules/master_machine/master_process.c
    modules/master_machine/seasky_protocol.c
    modules/BMI088/bmi088.c
    modules/imu/BMI088driver.c
    modules/imu/BMI088Middleware.c
    modules/imu/ins_task.c
    modules/vofa/vofa.c
    modules/motor/DJImotor/dji_motor.c
    modules/motor/DMmotor/dmmotor.c
    modules/motor/motor_task.c
    modules/daemon/daemon.c

    application/robot.c
    application/gimbal/gimbal.c
    application/cmd/robot_cmd.c
    application/cmd/opencv.c
    application/laser_ctrl/laser_proto.c
    application/laser_ctrl/galvanometer.c
    application/laser_ctrl/laser.c
    application/laser_ctrl/laser_task.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})

# SEGGER RTT asm uses preprocessor macros.
set_source_files_properties(
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s
    PROPERTIES COMPILE_OPTIONS "-x;assembler-with-cpp"
)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Drivers/CMSIS/DSP/Lib/GCC
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    c
    m
    nosys
    arm_cortexM7lfdp_math
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>"
    COMMAND ${CMAKE_OBJCOPY} -O ihex "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>" "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex"
    COMMAND ${CMAKE_OBJCOPY} -O binary -S "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>" "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin"
    COMMENT "Generating size, HEX and BIN outputs"
)
