cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# set(CMAKE_C_COMPILER armclang)  
# set(CMAKE_CXX_COMPILER armclang++)  
# set(CMAKE_ASM_COMPILER armasm)  


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME STM32G474_CLANG)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# 创建一个接口库
add_library(arm2d_includes INTERFACE)
# 将需要全局管理的路径附加到这个接口库上
target_include_directories(arm2d_includes INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Inc          # arm_math.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Arm-2D_Port/Cfg                     # arm_2d_cfg.h, rewrite cfg.h in user dir
)


set(ARM2D_HELPER ON)
set(ARM2D_LCD_PRINTF ON)
set(ARM2D_CONTROLS ON)

# 在引入ARM-2D之前设置CMSISCORE
set(CMSISCORE "${CMAKE_SOURCE_DIR}/Drivers/CMSIS" CACHE STRING "Path to CMSIS Core")
# 如果该变量已被缓存，且你想确保使用新值，可以加上FORCE关键字
# set(CMSISCORE "${CMAKE_SOURCE_DIR}/Drivers/CMSIS" CACHE STRING "Path to CMSIS Core" FORCE)
add_compile_definitions(__PERF_COUNTER__)  

# 然后添加ARM-2D的子目录
add_subdirectory(Arm-2D)
add_subdirectory(cmake/perf_counter)
# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)


# 在添加了 Arm-2D 子目录后，让其链接此接口库
target_link_libraries(ARM2D PUBLIC arm2d_includes)


# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    User/lcd.c
    User/LCD_1in3.c
    Arm-2D_Port/adapter/arm_2d_disp_adapter_0.c
)



# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    User/
    perf_counter/
    Arm-2D_Port/Cfg
    Arm-2D_Port/adapter
    Arm-2D/Library/Include
    Arm-2D/Helper/Include
)


# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    RTE_Acceleration_Arm_2D_Helper_Disp_Adapter0
    RTE_Acceleration_Arm_2D_Extra_LCD_printf
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    perf_counter
    ARM2D
    stm32cubemx
    # Add user defined libraries
)
