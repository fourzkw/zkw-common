cmake_minimum_required(VERSION 3.1)

project(camera_check)

find_package( OpenCV REQUIRED )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(${OpenCV_INCLUDE_DIRS})

aux_source_directory(./src SRC_LIST)

add_executable( camera_check ${SRC_LIST} )

target_link_libraries( camera_check ${OpenCV_LIBS} )