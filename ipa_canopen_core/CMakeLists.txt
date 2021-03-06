cmake_minimum_required(VERSION 2.8.3)
project(ipa_canopen_core)

find_package(catkin REQUIRED COMPONENTS roscpp libpcan)


FILE(GLOB ff "include/ipa_canopen_core/*.h*")
add_custom_target(qwelkjh   qe SOURCES ${ff})

if(CMAKE_COMPILER_IS_GNUCXX)
   execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
   if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
        message(STATUS "C++11 activated.")
        add_definitions("-std=gnu++11")
   elseif(GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3)
        message(WARNING "C++0x activated. If you get any errors update to a compiler which fully supports C++11")
        add_definitions("-std=gnu++0x")
   else ()
        message(FATAL_ERROR "C++11 needed. Therefore a gcc compiler with a version higher than 4.3 is needed.")
   endif()
else(CMAKE_COMPILER_IS_GNUCXX)
   add_definitions("-std=c++0x")
endif(CMAKE_COMPILER_IS_GNUCXX)

catkin_package(
    DEPENDS #
    CATKIN_DEPENDS roscpp libpcan
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########
include_directories (include ${catkin_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/)
#set(INCLUDE_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/)
  


###########
## src ##
###########
add_library(${PROJECT_NAME}
    src/device.cpp
    src/ipa_canopen_core.cpp
    src/motor.cpp
    src/encoder.cpp
    src/imu.cpp
    src/io_module.cpp
    src/device_group.cpp
  )
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
  
#list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs" )


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME} 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
