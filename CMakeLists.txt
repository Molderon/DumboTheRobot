cmake_minimum_required(VERSION 3.0)

project(System_Core)

set(SOURCE_FILES
    System_Core.cpp
    Space_Traversal.cpp
    Robot.cpp
    Error_Handlers.cpp
    Robot_Annimation.cpp
    Computer_Vision.cpp
)


set(HEADER_FILES
    Robot.hpp
)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(System_Core ${SOURCE_FILES} ${HEADER_FILES})


target_link_libraries(System_Core pthread czmq ${OpenCV_LIBS})

add_custom_target(clean-files
    COMMAND ${CMAKE_COMMAND} -P clean.cmake
    COMMENT "Cleaning generated files..."
)

file(WRITE clean.cmake "file(REMOVE \${CMAKE_BINARY_DIR}/System_Core)")

