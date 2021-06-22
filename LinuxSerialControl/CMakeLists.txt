cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(LinuxSerialControl)

set(CMAKE_CXX_STANDARD 11) 
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -Wall" )

set(COMMON_LIBS rt dl pthread)
message(STATUS "COMMON_LIBS=${COMMON_LIBS}")

#--------------------------------------------------------------------------------------

include_directories( ../libraries/SBGC_lib)
include_directories( ../libraries/SBGC_Linux)


# TARGET_LINK_LIBRARIES( LinuxSerialControl.cpp pthread)

add_executable( LinuxSerialControl
  LinuxSerialControl.cpp
  ../libraries/SBGC_lib/SBGC_cmd_helpers.cpp
  ../libraries/SBGC_Linux/SBGC_Linux.cpp
)

TARGET_LINK_LIBRARIES( LinuxSerialControl pthread)
#target_include_directories( uart
#  PUBLIC ${SPINNAKER_INCLUDE_DIR}
#)
#target_link_libraries( uart
#  PRIVATE ${OpenCV_LIBS}
#  ${SPINNAKER_LIBS}
#)

