cmake_minimum_required(VERSION 3.10)

# ========== C++ 标准 ==========
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 项目名
project(uav_controller
    VERSION 1.0
    LANGUAGES CXX
)

# ========== 编译类型 ==========
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ========== 编译选项 ==========
add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
)

# Debug 下加调试信息
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -Wextra -Wpedantic -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra -Wpedantic")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin/debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin/release)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
else()
    add_definitions(-DRELEASE)
endif()

find_package(PkgConfig REQUIRED)
# ========== 查找依赖 ==========
find_package(OpenCV REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)
pkg_check_modules(ZBAR REQUIRED zbar)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
# ========== 头文件路径 ==========
include_directories(
    "${PROJECT_SOURCE_DIR}/include"
    ${OpenCV_INCLUDE_DIRS}
    ${LIBSERIAL_INCLUDE_DIRS}
    ${ZBAR_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
)

# ========== 源文件 ==========
file(GLOB_RECURSE SRC_FILES
    src/*.cpp
)

# ========== 可执行文件 ==========
add_executable(${PROJECT_NAME}
    ${SRC_FILES}
)

# ========== 链接库 ==========
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    pthread
    ${OpenCV_LIBS}
    ${LIBSERIAL_LIBRARIES}
    ${ZBAR_LIBRARIES}
    ${JSONCPP_LIBRARIES}
)

# ========== 兼容性设置 ==========
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9.1")
    target_link_libraries(${PROJECT_NAME} PRIVATE stdc++fs)
endif()

# ========== 输出信息 ==========
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "ZBar version: ${ZBAR_VERSION}")
message(STATUS "JsonCpp version: ${JSONCPP_VERSION}")
message(STATUS "LibSerial version: ${LIBSERIAL_VERSION}")

