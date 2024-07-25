cmake_minimum_required(VERSION 3.0.2)
project(vo)

set (CMAKE_CXX_FLAGS "-lstdc++fs -std=c++17")

set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV 4.6.0 REQUIRED)
find_package(Eigen3 3.3.4 REQUIRED)
find_package(Ceres 2.2.0 EXACT REQUIRED)
find_package(Sophus 1.22.10 REQUIRED)

include_directories(
    include
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/examples
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Sophus_INCLUDE_DIRS}
)

add_executable(
    bundle_adjuster_on_manifold
    bundle_adjuster_on_manifold.cc
)
target_link_libraries(
    bundle_adjuster_on_manifold 
    ${CERES_LIBRARIES} 
    ${EIGEN3_LIBS}
    Sophus::Sophus
)

add_executable(
    frontend
    src/frontend.cc
    src/Utils.cc
    src/Feature.cc
)
target_link_libraries(
    frontend 
    ${OpenCV_LIBRARIES} 
    ${EIGEN3_LIBS}
)