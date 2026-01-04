cmake_minimum_required(VERSION 3.15)
project(perf_counter)

# 设置C标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMSISCORE "" CACHE STRING "Path to CMSIS Core")

# 定义源文件列表（清晰明了）
set(PERF_COUNTER_SOURCES
    ../../perf_counter/perf_counter.c
    ../../perf_counter/perfc_port_default.c
    ../../perf_counter/perfc_port_pmu.c
    ../../perf_counter/perfc_task_coroutine.c
)

# 创建库并直接指定源文件
add_library(perf_counter STATIC ${PERF_COUNTER_SOURCES})

target_include_directories(perf_counter PUBLIC ${CMSISCORE}/Include)

# 设置包含目录
target_include_directories(perf_counter 
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
    PRIVATE 
        ../../perf_counter/
)

# 可选：设置目标属性
set_target_properties(perf_counter PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS ON
)