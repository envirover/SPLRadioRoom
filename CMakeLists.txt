cmake_minimum_required(VERSION 3.0.0)

project(radioroom)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra" )

include_directories("${PROJECT_SOURCE_DIR}/libs/mavlink/include/standard")

file(GLOB sources src/*.c src/*.cc)

add_executable(radioroom ${sources})

install(TARGETS radioroom DESTINATION "/usr/sbin")
install(DIRECTORY "${PROJECT_SOURCE_DIR}/etc/" DESTINATION "/etc")

set(CPACK_GENERATOR "ZIP")
set(CPACK_PACKAGE_NAME "radioroom")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Pavel Bobov")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "SPL RadioRoom")
set(CPACK_PACKAGE_VENDOR "Envirover")
#SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ReadMe.txt")
#SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/Copyright.txt")
set(CPACK_PACKAGE_VERSION_MAJOR "2")
set(CPACK_PACKAGE_VERSION_MINOR "0")
set(CPACK_PACKAGE_VERSION_PATCH "0")

include(CPack)

