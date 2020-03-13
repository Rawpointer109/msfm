cmake_minimum_required(VERSION 3.0)
project(multi_sensor_fusion_mapping)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Release)

# if any conan libraries are used
# set(CONAN_BUILD_INFO_CMAKE ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
# if(NOT EXISTS ${CONAN_BUILD_INFO_CMAKE})
#     message(FATAL_ERROR "Conan Library Not Installed!")
# endif()
# include(${CONAN_BUILD_INFO_CMAKE})
# conan_basic_setup()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL REQUIRED)
add_definitions(${PCL_DIFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_libraries(${PCL_LIBRARY_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/src
                    ${PROJECT_SOURCE_DIR}/src/feature
                    ${PROJECT_SOURCE_DIR}/src/frame
                    ${PROJECT_SOURCE_DIR}/src/geometry
                    ${PROJECT_SOURCE_DIR}/third_party/nlohmann)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set(HEADER_ROOT ${PROJECT_SOURCE_DIR}/src)
file(GLOB_RECURSE HEADERS 
     src/feature/*.h 
     src/frame/*.h 
     src/geometry/*.h
     src/geometry/*.hpp
     src/map/*.h
     third_party/nlohmann/*.hpp)
file(GLOB_RECURSE SOURCES
     src/feature/*.cpp 
     src/frame/*.cpp
     src/geometry/*.cpp
     src/map/*.cpp)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

add_executable(mapper src/main.cpp)
target_link_libraries(mapper ${PROJECT_NAME} ${PCL_LIBRARIES})
