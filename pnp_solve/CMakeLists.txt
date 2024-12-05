cmake_minimum_required(VERSION 3.1)

project( pnp_solve )

find_package( OpenCV REQUIRED )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories(./inc)

aux_source_directory(./src SRC_LIST)

add_executable( pnp_solve ${SRC_LIST} )

target_link_libraries( pnp_solve ${OpenCV_LIBS} )
