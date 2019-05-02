cmake_minimum_required(VERSION 2.8.12)
project(argos_sferes)

set(SFERES ${PROJECT_SOURCE_DIR}/sferes2) # location of sferes2 - where https://github.com/sferes2/sferes2 has been cloned
set(THYMIO $ENV{THYMIO}) # location of Thymio plugin installation -- include and libs
IF( NOT THYMIO )
    MESSAGE( FATAL_ERROR "Please point the environment variable THYMIO to the location of the Thymio plugin installation")
ENDIF()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Deactivate RPATH for MacOSX
#set(CMAKE_MACOSX_RPATH 0)

# Set the path where CMake will find additional scripts
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# Find the ARGoS package
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
list(APPEND CMAKE_MODULE_PATH ${ARGOS_PREFIX}/share/argos3/cmake)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(Boost_NO_BOOST_CMAKE      ON) # disable the search for boost-cmake
set(Boost_USE_MULTITHREADED      ON)
set(BOOST_INCLUDEDIR "/usr/include/boost/") # hint location to search for boost headers
set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu/") # hint location to search for boost libs
#find_package(Boost COMPONENTS serialization filesystem system unit_test_framework program_options graph mpi python thread REQUIRED)
#find_package(Boost COMPONENTS serialization filesystem system unit_test_framework program_options graph thread REQUIRED)
find_package(Boost COMPONENTS serialization graph program_options filesystem  REQUIRED)
# system unit_test_framework thread REQUIRED)
if(Boost_FOUND)
  link_directories(${Boost_LIBRARY_DIRS})
  include_directories(${Boost_INCLUDE_DIRS})
  message("Boost libs found in: " ${Boost_LIBRARY_DIRS}) 
  message("Boost headers found in: " ${Boost_INCLUDE_DIRS}) 
else(NOT Boost_FOUND)
    message(FATAL_ERROR "Unable to continue without BOOST")
endif()

#Find TBB libs -- script from https://github.com/justusc/FindTBB
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/FindTBB)
find_package(TBB) #COMPONENTS tbbmalloc tbbmalloc_proxy 
if(TBB_FOUND)
  include_directories(${TBB_INCLUDE_DIRS})
  message(${TBB_INCLUDE_DIRS} " " ${TBB_LIBRARIES})
else(NOT TBB_FOUND)
  message("Unable to find TBB. Will not be able to run sferes eval::Parallel")
  set(TBB_INCLUDE_DIRS "")
  set(TBB_LIBRARIES "")
endif()


#link_directories(/usr/lib) ## careless -- which libs are we trying to link with?


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
#add_definitions(-std=c++0x)
add_definitions( -DEIGEN3_ENABLED )
if(NOT TBB_FOUND)
  add_definitions( -DNO_PARALLEL )
endif()
add_definitions( -DNDEBUG ) # non-debug mode
add_definitions( -DVERSION="\(const char*\)\\"f731bfb04a48a475dfedbfdb180d77054498557f\\"" ) #value not important. added to prevent compilation error in sferes/run.hpp




#include_directories(/usr/include)  ## careless -- exactly which includes?
#include_directories("/usr/include/lua5.2")  # think we can do without lua
SET(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
IF(NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
include_directories("${EIGEN3_INCLUDE_DIR}")

include_directories(${SFERES})
include_directories($ENV{HOME}/geometric-median) 
include_directories(${THYMIO}/include)
message("Thymio path is: " ${THYMIO}) 
include_directories(${TBB_INCLUDE_DIRS})



# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
include(ARGoSCheckQTOpenGL)

# Find the GSL package
find_package(GSL REQUIRED)

# Find Lua
#find_package(Lua52 REQUIRED) #  think we can do without lua

# Set ARGoS include dir
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${GSL_INCLUDE_DIR}) #${LUA_INCLUDE_DIR})

# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS} ${THYMIO}/lib/argos3 ${PROJECT_SOURCE_DIR}/lib)

# Descend into the controllers directory

add_library(obsavoid_nn_controller SHARED
  src/obsavoid/nn_controller.h
  src/obsavoid/nn_controller.cpp
)

add_library(descriptors SHARED
  src/obsavoid/descriptors.h
  src/obsavoid/descriptors.cpp
)

add_library(fitness_functions SHARED
  src/obsavoid/fitness_functions.h
  src/obsavoid/fitness_functions.cpp
)

add_library(statistics SHARED
  src/obsavoid/statistics.h
  src/obsavoid/statistics.cpp
)

add_library(obsavoid_evol_loopfunctions SHARED
  src/obsavoid/evol_loop_functions.h
  src/obsavoid/evol_loop_functions.cpp
)

option(DEFINE_PRINT "print some variables" ON)
option(3D_BD "3D behaviour description" OFF)
option(6D_BD "6D behaviour description" OFF)
option(42D_BD "42D behaviour description" OFF)
option(150D_BD "150D behaviour description" OFF)
option(CVT_USAGE "Centroidal Voronoi Tessellation MAP-Elites" OFF)

if(DEFINE_PRINT)
    message("print is defined")
    add_definitions(-DPRINTING)    
else(NOT DEFINE_PRINT)
    message("print is NOT defined")
endif()

if(3D_BD)
  set(BD_SIZE 3)
  add_definitions(-DTHREE_D_BEHAV)
  add_executable(obsavoid_evol3D src/obsavoid/main.cpp)
elseif(6D_BD)
  set(BD_SIZE 6)
  add_definitions(-DSIX_D_BEHAV)
  message("Using 6D behavioural characterisation")
elseif(42D_BD)
  set(BD_SIZE 42)
  add_definitions(-DFOURTYTWO_D_BEHAV)
  message("Using 42D behavioural characterisation")
elseif(150D_BD)
  set(BD_SIZE 150)
  add_definitions(-DHUNDREDFIFTY_D_BEHAV)
  message("Using 150D behavioural characterisation")
else()
  set(BD_SIZE 3)
  add_definitions(-DTHREE_D_BEHAV)
  message("Using 3D behavioural characterisation")
endif()


if(CVT_USAGE)
  set(CVT_USE cvt)
  add_definitions(-DCVT)
  message("Using Centroidal Voronoi Tesselation")
endif()

set(EXECUTABLE obsavoid_evol${CVT_USE}${BD_SIZE}D)
add_executable(${EXECUTABLE} src/obsavoid/main.cpp)

target_link_libraries(obsavoid_nn_controller
  argos3core_simulator
  #argos3plugin_simulator_actuators
  argos3plugin_simulator_thymio
  argos3plugin_simulator_genericrobot
  #argos3plugin_simulator_sensors
  ${Boost_LIBRARIES}
  ${TBB_LIBRARIES})

target_link_libraries(obsavoid_evol_loopfunctions
  obsavoid_nn_controller
  argos3core_simulator
  #argos3plugin_simulator_actuators
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_thymio
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  #argos3plugin_simulator_sensors
  ${Boost_LIBRARIES}
  ${TBB_LIBRARIES})

target_link_libraries(${EXECUTABLE}
  obsavoid_nn_controller
  obsavoid_evol_loopfunctions
  descriptors
  fitness_functions
  statistics
  argos3core_simulator 
  ${Boost_LIBRARIES} 
  ${TBB_LIBRARIES})