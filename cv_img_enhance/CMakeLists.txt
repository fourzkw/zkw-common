cmake_minimum_required(VERSION 3.1)

project(cv_img_enhance)

find_package( OpenCV REQUIRED )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(${OpenCV_INCLUDE_DIRS})

aux_source_directory(./src SRC_LIST)

add_executable( img_enhance ${SRC_LIST} )

target_link_libraries( img_enhance ${OpenCV_LIBS} )