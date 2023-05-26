cmake_minimum_required(VERSION 3.13)

project(rCore C CXX)

cmake_policy(VERSION 3.13)
cmake_policy(SET CMP0077 NEW)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake;${HOME}/.local/lib;${CMAKE_MODULE_PATH}")
# include(cmake/common.cmake)

# Target Art Protocol
add_subdirectory(src)

# Targets
foreach(_target test_rcore)
  add_executable(
    ${_target}
    "${_target}.cpp"
  )
  target_link_libraries(
    ${_target}
    rcore
  )
  target_include_directories(
    ${_target}
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
endforeach()