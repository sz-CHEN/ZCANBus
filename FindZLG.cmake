if(CMAKE_HOST_WIN32)
find_path(ZLG_INCLUDE_DIR controlcan.h PATH_SUFFIXES ZLG)
find_library(ZLG_LIBS NAMES usbcan PATH_SUFFIXES MS x64 x86 win32)
elseif(CMAKE_HOST_UNIX)
find_path(ZLG_INCLUDE_DIR NAMES controlcan.h PATH_SUFFIXES ZLG)
find_library(ZLG_LIBS NAMES usbcan controlcan)
else()
message(FATAL_ERROR "Unsupported OS " ${CMAKE_SYSTEM_NAME})
endif(CMAKE_HOST_WIN32)
if(ZLG_INCLUDE_DIR AND ZLG_LIBS)
set(ZLG_FOUND TRUE)
message(STATUS "Found ZLG")
else()
unset(ZLG_FOUND)
set(ZLG_INCLUDE_DIR)
set(ZLG_LIBS)
if(ZLG_FIND_REQUIRED)
message(FATAL_ERROR  "Not found ZLG")
else()
message(WARNING "Not found ZLG")
endif()
endif()