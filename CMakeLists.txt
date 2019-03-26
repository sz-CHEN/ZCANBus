cmake_minimum_required(VERSION 3.0)
project(ZCANBus)

set(CMAKE_CXX_STANDARD 11)

# set(USE_PEAK true)

# set(USE_KVASER true)

# set(USE_ZLG true)

# set(USE_ZLG2 true)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(_COMPILER_ARCHITECTURE x64)
endif(CMAKE_SIZEOF_VOID_P EQUAL 8)

if(MSVC)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()
set(CAN_FILE CANHandler.cpp)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Peak)
find_package(Kvaser)
find_package(ZLG)
find_package(ZLG2)

if(Peak_FOUND)
add_definitions(-DUSE_PEAK)
list(APPEND CAN_FILE CANPeak.cpp)
include_directories(${Peak_INCLUDE_DIR})
link_libraries(${Peak_LIBS})
endif()

if(Kvaser_FOUND)
add_definitions(-DUSE_KVASER)
list(APPEND CAN_FILE CANKvaser.cpp)
include_directories(${Kvaser_INCLUDE_DIR})
link_libraries(${Kvaser_LIBS})
endif()

if(ZLG_FOUND)
add_definitions(-DUSE_ZLG)
list(APPEND CAN_FILE CANZLG.cpp)
include_directories(${ZLG_INCLUDE_DIR})
link_libraries(${ZLG_LIBS})
endif()

if(ZLG2_FOUND)
add_definitions(-DUSE_ZLG2)
list(APPEND CAN_FILE CANZLG2.cpp)
include_directories(${ZLG2_INCLUDE_DIR})
link_libraries(${ZLG2_LIBS})
endif()

if(CMAKE_HOST_UNIX)
  link_libraries(pthread)
endif(CMAKE_HOST_UNIX)

set(CMAKE_DEBUG_POSTFIX _d)
add_library(${PROJECT_NAME} ${CAN_FILE})
