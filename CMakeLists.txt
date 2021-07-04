cmake_minimum_required(VERSION 3.0.0)

project(radioroom VERSION 2.5.0)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(CPACK_GENERATOR "DEB")
# set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "armhf")
set(CPACK_PACKAGE_NAME "radioroom")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Pavel Bobov")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Telemetry for MAVLink autopilots")
set(CPACK_PACKAGE_VENDOR "Envirover")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_VERSION_MAJOR "${PROJECT_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${PROJECT_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${PROJECT_VERSION_PATCH}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++11 -lpthread" )

configure_file("${PROJECT_SOURCE_DIR}/src/build.h.in" "${PROJECT_BINARY_DIR}/build.h")

include_directories("${PROJECT_SOURCE_DIR}/libs/mavlink/include")
include_directories("${PROJECT_SOURCE_DIR}/libs/mavio/include")
include_directories("${PROJECT_SOURCE_DIR}/libs/inih/include")
include_directories("${PROJECT_SOURCE_DIR}/libs/timelib/include")
include_directories("${PROJECT_BINARY_DIR}")

file(GLOB mavio_sources libs/mavio/src/*.cc)
file(GLOB inih_sources libs/inih/src/*.cc libs/inih/src/*.c)
file(GLOB sources src/*.c src/*.cc)

add_library(mavio STATIC ${mavio_sources})
add_library(inih STATIC ${inih_sources})

add_executable(radioroom ${sources})
target_link_libraries(radioroom mavio)
target_link_libraries(radioroom inih)

add_executable(aptest tests/MAVLinkAutopilotTest.cc)
add_executable(tltest tests/TimeLibTest.cc)
add_executable(cbtest tests/CircularBufferTest.cc)
add_executable(tcptest tests/MAVLinkTCPChannelTest.cc)
add_executable(isbdtest tests/MAVLinkISBDChannelTest.cc)
add_executable(srtest tests/CustomSerializationTest.cc)

target_link_libraries(aptest mavio)
target_link_libraries(cbtest mavio)
target_link_libraries(tcptest mavio)
target_link_libraries(isbdtest mavio)
target_link_libraries(srtest mavio)

install(TARGETS radioroom DESTINATION "/usr/sbin")

# Install files common for all systems
install(DIRECTORY "${PROJECT_SOURCE_DIR}/pack/common/etc/" DESTINATION "/etc" 
        FILE_PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ)

# Install system-specific files
install(DIRECTORY "${PROJECT_SOURCE_DIR}/pack/${CMAKE_SYSTEM_NAME}/etc/" DESTINATION "/etc" 
        FILE_PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ)

include(CPack)
