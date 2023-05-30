cmake_minimum_required(VERSION 3.13)

SET(EIGEN3_ROOT_DIR "$ENV{EIGEN3_ROOT}")
IF(NOT EIGEN3_ROOT_DIR)
    MESSAGE(FATAL_ERROR "Please point the environment variable EIGEN3_ROOT to the include directory of your Eigen3 installation.")
ENDIF()
find_package(Eigen3 3.4.0 REQUIRED)
if(NOT Eigen3_FOUND)
    MESSAGE(FATAL_ERROR "Eigen3 not found.")
endif()
MESSAGE(STATUS "Eigen3 was found at ${EIGEN3_INCLUDE_DIR}")
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

file(GLOB SRCS "*.cpp")
file(GLOB HDRS "*.h")

# Targets
add_library(rcore
    Interpolator.cpp
    rxInterpolator.cpp
#    ${SRCS}
#    ${HDRS}
)
# target_link_libraries(
#     rcore
# )
target_include_directories(
    rcore
    PUBLIC
#    "/usr/include/eigen3"
    "${EIGEN3_INCLUDE_DIR}"
    "${CMAKE_SOURCE_DIR}/include"
)
