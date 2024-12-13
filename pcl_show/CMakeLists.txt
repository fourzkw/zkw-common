cmake_minimum_required(VERSION 2.6)
project(pcl_show)

find_package(PCL REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

aux_source_directory(./src SRC_LIST)

add_executable(pcl_show ${SRC_LIST})

target_link_libraries (pcl_show ${PCL_LIBRARIES})

target_link_libraries(pcl_show ${Boost_LIBRARIES})

install(TARGETS pcl_show RUNTIME DESTINATION bin)

