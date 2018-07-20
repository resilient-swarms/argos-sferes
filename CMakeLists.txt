cmake_minimum_required(VERSION 2.8.12)
project(epuck_obsavoid_nn)

set(SFERES_PATH ${PROJECT_SOURCE_DIR}/sferes2) # location of sferes2 - where https://github.com/sferes2/sferes2 has been cloned

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
endif()

# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)

# Set the path where CMake will find additional scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Find the ARGoS package
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(Boost_NO_BOOST_CMAKE      ON)
set(Boost_USE_MULTITHREADED      ON)
set(BOOST_INCLUDEDIR "/usr/include/boost/") # hint location to search for boost libs
set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu/") # hint location to search for boost libs
find_package(Boost COMPONENTS serialization filesystem system unit_test_framework program_options graph mpi python thread REQUIRED)
if(Boost_FOUND)
  link_directories(${Boost_LIBRARY_DIRS})
  include_directories(${Boost_INCLUDE_DIRS})
endif()

link_directories(/usr/lib)

add_definitions(-std=c++0x)
add_definitions( -DEIGEN3_ENABLED )
add_definitions( -DNO_PARALLEL )
add_definitions( -DNDEBUG )
add_definitions( -DVERSION="\(const char*\)\\"f731bfb04a48a475dfedbfdb180d77054498557f\\"" ) #value not important. added to prevent compilation error in sferes/run.hpp

include_directories(/usr/include)
include_directories(/usr/include/eigen3)
include_directories(${SFERES_PATH})


# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
include(ARGoSCheckQTOpenGL)

# Find the GSL package
find_package(GSL REQUIRED)

# Find Lua
find_package(Lua52 REQUIRED)

# Set ARGoS include dir
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${GSL_INCLUDE_DIR} ${LUA_INCLUDE_DIR})

# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS} ${PROJECT_SOURCE_DIR}/lib)

# Descend into the controllers directory

add_library(epuck_nn SHARED
  src/epuck_nn_controller.h
  src/epuck_nn_controller.cpp
)

add_library(evolution_loop_functions SHARED
  src/evolution_loop_functions.h
  src/evolution_loop_functions.cpp
)

add_executable(argos_galib src/main.cpp)


if(Boost_FOUND)
    target_link_libraries(epuck_nn
      argos3core_simulator
      #argos3plugin_simulator_actuators
      argos3plugin_simulator_epuck
      argos3plugin_simulator_genericrobot
      #argos3plugin_simulator_sensors
      ${Boost_LIBRARIES})

    target_link_libraries(evolution_loop_functions
      epuck_nn
      argos3core_simulator
      #argos3plugin_simulator_actuators
      argos3plugin_simulator_dynamics2d
      argos3plugin_simulator_entities
      argos3plugin_simulator_epuck
      argos3plugin_simulator_genericrobot
      argos3plugin_simulator_media
      #argos3plugin_simulator_sensors
      ${Boost_LIBRARIES})

    target_link_libraries(argos_galib
      epuck_nn
      evolution_loop_functions
      argos3core_simulator ${Boost_LIBRARIES})

else(NOT Boost_FOUND)

 
endif()
