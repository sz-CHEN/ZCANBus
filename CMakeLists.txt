cmake_minimum_required(VERSION 3.0)
project(ZCANBus)

set(CMAKE_CXX_STANDARD 11)

# set(PEAK_WANTED true)

# set(KVASER_WANTED true)

# set(ZLG_WANTED true)

# set(ZLG2_WANTED true)

# set(PEAK_UNWANTED true)

# set(KVASER_UNWANTED true)

# set(ZLG_UNWANTED true)

# set(ZLG2_UNWANTED true)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(_COMPILER_ARCHITECTURE x64)
endif(CMAKE_SIZEOF_VOID_P EQUAL 8)

if(MSVC)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()
set(CAN_FILE CANHandler.cpp)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

if(PEAK_WANTED)
set(PEAK_REQUIRED REQUIRED)
endif()
if(KVASER_WANTED)
set(KVASER_REQUIRED REQUIRED)
endif()
if(ZLG_WANTED)
set(ZLG_REQUIRED REQUIRED)
endif()
if(ZLG2_WANTED)
set(ZLG2_REQUIRED REQUIRED)
endif()

if(NOT PEAK_UNWANTED)
find_package(Peak ${PEAK_REQUIRED})
endif()
if(NOT KVASER_UNWANTED)
find_package(Kvaser ${KVASER_REQUIRED})
endif()
if(NOT ZLG_UNWANTED)
find_package(ZLG ${ZLG_REQUIRED})
endif()
if(NOT ZLG2_UNWANTED)
find_package(ZLG2 ${ZLG2_REQUIRED})
endif()

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

# set(CMAKE_DEBUG_POSTFIX _d)
add_library(${PROJECT_NAME} ${CAN_FILE})
