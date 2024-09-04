cmake_minimum_required(VERSION 3.10)
project(SerialCommEncoder)

set(CMAKE_CXX_STANDARD 17)

find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}
    SerialCommEncoder.hpp
    SerialCommEncoder.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC pthread yaml-cpp::yaml-cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
