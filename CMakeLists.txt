cmake_minimum_required(VERSION 3.0)
project(ZCANBus)

set(CMAKE_CXX_STANDARD 11)

if(NOT CMAKE_HOME_DIRECTORY)
    set(CMAKE_HOME_DIRECTORY "./")
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(_COMPILER_ARCHITECTURE x64)
endif(CMAKE_SIZEOF_VOID_P EQUAL 8)

if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

set(USE_PEAK true)
set(USE_KVASER)
set(CAN_FILE CANHandler.cpp)

if(USE_PEAK)
    list(APPEND CAN_FILE CANPeak.cpp)
    if(CMAKE_HOST_WIN32)
        include_directories(include/peak)
        link_libraries(PCANBasic)
    elseif(CMAKE_HOST_APPLE)
        link_libraries(PCBUSB)
    elseif(CMAKE_HOST_UNIX)
        link_libraries(pcanbasic)
    endif(CMAKE_HOST_WIN32)
    add_definitions(-DUSE_PEAK)
endif(USE_PEAK)

if(USE_KVASER)
    list(APPEND CAN_FILE CANKvaser.cpp)
    if(CMAKE_HOST_WIN32)
        include_directories(include/kvaser)
        link_libraries(canlib32)
    elseif(CMAKE_HOST_UNIX)
        link_libraries(canlib)
    endif(CMAKE_HOST_WIN32)
    add_definitions(-DUSE_KVASER)
endif(USE_KVASER)

if(CMAKE_HOST_WIN32)
    if(_COMPILER_ARCHITECTURE STREQUAL x64)
        link_directories(lib/x64)
    else()
        link_directories(lib/MS)
    endif(_COMPILER_ARCHITECTURE STREQUAL x64)
endif(CMAKE_HOST_WIN32)

if(CMAKE_HOST_UNIX)
    link_libraries(pthread)
endif(CMAKE_HOST_UNIX)

set(CMAKE_DEBUG_POSTFIX _d)

add_library(${PROJECT_NAME} ${CAN_FILE})