cmake_minimum_required(VERSION 3.1)

project(cv_color)

find_package( OpenCV REQUIRED )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(inc)

aux_source_directory(./src SRC_LIST)

add_executable( colorDetector ${SRC_LIST} )

target_link_libraries( colorDetector ${OpenCV_LIBS} )