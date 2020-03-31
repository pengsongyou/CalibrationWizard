set(PROJECT_NAME CalibrationWizard)
project(${PROJECT_NAME})
cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 11)

set(EXECUTABLE_OUTPUT_PATH bin)
file(MAKE_DIRECTORY "out/images/ini_calib")
file(MAKE_DIRECTORY "out/images/nextpose")

# add OpenCV library
find_package( OpenCV REQUIRED )

# add source files
file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} )
message(STATUS "OpenCV Found in ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Source files includes ${SOURCE_FILES}")

