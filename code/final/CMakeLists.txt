cmake_minimum_required(VERSION 3.0)
project(main)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

find_path(CURL_INCLUDE_DIR NAMES curl/curl.h PATHS ${CURL_DIR}/include)
find_library(CURL_LIBRARY NAMES libcurl PATHS ${CURL_DIR}/lib)

# find_package(CURL REQUIRED)
include_directories(${CURL_INCLUDE_DIR})

if (!WIN32)
    link_libraries(pthread rt m)
endif (!WIN32)

aux_source_directory(. src)
add_executable(main ${src})

target_link_libraries(main ${CURL_LIBRARY})

add_definitions(-DDEBUG)
add_definitions(-DLOG)
add_definitions(-Wall)

# 设置生成的可执行文件的输出目录
set_target_properties(main PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/SDK/MySolution"
)