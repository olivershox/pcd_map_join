cmake_minimum_required(VERSION 3.5)
project(dump_map)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall")

find_package(PCL REQUIRED)

#find_package(Sophus REQUIRED)
include_directories(${Sophus_DIRECTORIES})
include_directories("/usr/include/eigen3")

include_directories(${PROJECT_SOURCE_DIR}/src)

add_library(${PROJECT_NAME}
	       src/keyframe.cpp)
target_link_libraries(${PROJECT_NAME} fmt glog gflags )
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

	       
	       
include_directories(${Sophus_DIRECTORIES})
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PCL_INCLUDE_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)


add_executable(map src/dump_map.cpp)
target_link_libraries(map fmt glog gflags )

target_link_libraries(map ${PCL_LIBRARIES})
target_link_libraries(map ${PROJECT_NAME})
