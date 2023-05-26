cmake_minimum_required(VERSION 3.13)

file(GLOB SRCS "*.cpp")
file(GLOB HDRS "*.h")

# Targets
add_library(rcore
    Interpolator.cpp
#    ${SRCS}
#    ${HDRS}
)
# target_link_libraries(
#     rcore
# )
target_include_directories(
    rcore
    PRIVATE
    "/usr/include/eigen3"
    "${CMAKE_SOURCE_DIR}/include"
)
