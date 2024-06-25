cmake_minimum_required(VERSION 3.10)
project(roi_detector)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenMP REQUIRED)
include_directories("/opt/homebrew/opt/libomp/include")
link_directories("/opt/homebrew/opt/libomp/lib")

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/test)

set(HDRS ${CMAKE_SOURCE_DIR}/include/boundary_detector/boundary_detector.h
        ${CMAKE_SOURCE_DIR}/include/boundary_detector/closed_boundary_detector.h
        ${CMAKE_SOURCE_DIR}/include/boundary_extractor/boundary_extractor.h
        ${CMAKE_SOURCE_DIR}/include/boundary_extractor/closed_boundary_extractor.h
        ${CMAKE_SOURCE_DIR}/include/boundary_order_generator/boundary_order_generator.h
        ${CMAKE_SOURCE_DIR}/include/boundary_order_generator/crust_boundary_order_generator.h
        ${CMAKE_SOURCE_DIR}/include/boundary_order_generator/kdtree_boundary_order_generator.h
        ${CMAKE_SOURCE_DIR}/include/boundary_order_generator/tsp_boundary_order_generator.h
        ${CMAKE_SOURCE_DIR}/include/region_interpolator/mls_region_interpolator.h
        ${CMAKE_SOURCE_DIR}/include/region_interpolator/poisson_region_interpolator.h
        ${CMAKE_SOURCE_DIR}/include/region_interpolator/region_interpolator.h
        ${CMAKE_SOURCE_DIR}/include/types.h
        ${CMAKE_SOURCE_DIR}/include/utils.h)

set(SRCS ${CMAKE_SOURCE_DIR}/src/boundary_detector/closed_boundary_detector.cpp
        ${CMAKE_SOURCE_DIR}/src/boundary_extractor/closed_boundary_extractor.cpp
        ${CMAKE_SOURCE_DIR}/src/boundary_order_generator/crust_boundary_order_generator.cpp
        ${CMAKE_SOURCE_DIR}/src/boundary_order_generator/kdtree_boundary_order_generator.cpp
        ${CMAKE_SOURCE_DIR}/src/boundary_order_generator/tsp_boundary_order_generator.cpp
        ${CMAKE_SOURCE_DIR}/src/region_interpolator/mls_region_interpolator.cpp
        ${CMAKE_SOURCE_DIR}/src/region_interpolator/poisson_region_interpolator.cpp
        ${CMAKE_SOURCE_DIR}/src/utils.cpp
        ${CMAKE_SOURCE_DIR}/test/isu_roi_detection.cpp)

add_executable(roi_detector ${HDRS} ${SRCS})
target_link_libraries(roi_detector ${PCL_LIBRARIES})
target_link_libraries(roi_detector OpenMP::OpenMP_CXX)
