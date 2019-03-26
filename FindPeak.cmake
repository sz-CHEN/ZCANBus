if(CMAKE_HOST_WIN32)
    find_path(Peak_INCLUDE_DIR NAMES PCANBasic.h PATH_SUFFIXES peak)
    find_library(Peak_LIBS NAMES PCANBasic PATH_SUFFIXES MS x64 x86 win32)
elseif(CMAKE_HOST_UNIX)
    if(CMAKE_HOST_APPLE)
        find_path(Peak_INCLUDE_DIR NAMES PCBUSB.h PATH_SUFFIXES peak)
        find_library(Peak_LIBS NAMES PCBUSB)
    else()
        find_path(Peak_INCLUDE_DIR NAMES PCANBasic.h PATH_SUFFIXES peak)
        find_library(Peak_LIBS NAMES pcanbasic)
    endif(CMAKE_HOST_APPLE)
endif(CMAKE_HOST_WIN32)
if(Peak_INCLUDE_DIR AND Peak_LIBS)
    set(Peak_FOUND TRUE)
    message(STATUS "Found Peak")
else()
    unset(Peak_FOUND)
    unset(Peak_INCLUDE_DIR)
    unset(Peak_LIBS)
    if(Peak_FIND_REQUIRED)
    message(FATAL_ERROR  "Not found Peak")
    else()
    message(WARNING "Not found Peak")
    endif()
endif()