cmake_minimum_required(VERSION 3.10)
project(motion_control_pure)
set(CMAKE_CXX_STANDARD 17)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
message("PYTHON_EXECUTABLE: ${PYTHON_EXECUTABLE}")
message("Python3_EXECUTABLE: ${Python3_EXECUTABLE}")

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(yaml-cpp REQUIRED)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)
if(GeographicLib_FOUND)
    message(STATUS "GeographicLib found!")
else()
    message(FATAL_ERROR "GeographicLib not found!")
endif()


add_subdirectory(autonomous_proto)
include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/autonomous_proto)

add_executable(motion_control_pure
            src/main.cpp
            src/pure_pursuit.cpp
            include/pure_puresuit.hpp
)

ament_target_dependencies(motion_control_pure
        rclcpp
        std_msgs
)

target_include_directories(motion_control_pure PUBLIC
        autonomous_proto
        include
        src
        ${GeographicLib_INCLUDE_DIRS}
        yaml-cpp
)
target_link_libraries(motion_control_pure
        ${rclcpp_LIBRARIES}
        autonomous_proto
        Eigen3::Eigen
        ${GeographicLib_LIBRARIES}
        yaml-cpp
)

install(TARGETS motion_control_pure
        DESTINATION lib/${PROJECT_NAME}
)

ament_package( )


