cmake_minimum_required(VERSION 3.0)

# Set the project name
PROJECT(OpenCV-Camera-Calib)

add_definitions(-DPROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DRES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/res")
add_definitions(-DRESULT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/result")

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(yaml-cpp REQUIRED)
FIND_PACKAGE(spdlog REQUIRED)

# Add the include directories for the OpenCV library
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

ADD_LIBRARY(calib-lib
    src/OpenCVCamCalib.cpp
    src/OpenCVCamCalib.hpp
)

# Link the OpenCV library
TARGET_LINK_LIBRARIES(calib-lib PUBLIC
    ${OpenCV_LIBS}
    yaml-cpp::yaml-cpp
    spdlog::spdlog
)

# Add the executable
ADD_EXECUTABLE(calib
    src/calib.cpp
)

TARGET_LINK_LIBRARIES(calib PRIVATE
    calib-lib
)

ADD_EXECUTABLE(frame_aquisition
    src/frame_aquisition.cpp
)

TARGET_LINK_LIBRARIES(frame_aquisition PRIVATE
    calib-lib
)
