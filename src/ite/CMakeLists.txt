cmake_minimum_required(VERSION 2.6)
project(ite_swarms)

set(LIMBO_PATH /home/danesh/argos-limbo/limbo/src) # location of limbo - where https://github.com/resibots/limbo.git branch release-2.1 has been cloned
include_directories(${LIMBO_PATH})


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

SET(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
IF(NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ELSE()
    include_directories("${EIGEN3_INCLUDE_DIR}")
    add_definitions( -DEIGEN3_ENABLED )
ENDIF()

#Find TBB libs -- script from https://github.com/justusc/FindTBB
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}) # location of FindTBB
find_package(TBB) #COMPONENTS tbbmalloc tbbmalloc_proxy
if(TBB_FOUND)
  include_directories(${TBB_INCLUDE_DIRS})
  message("TBB headers found in: " ${TBB_INCLUDE_DIRS})
  message("TBB libs: " ${TBB_LIBRARIES})
  add_definitions(-DUSE_TBB)
else(NOT TBB_FOUND)
  message("Unable to find TBB. Will not be able to run sferes eval::Parallel")
  set(TBB_INCLUDE_DIRS "")
  set(TBB_LIBRARIES "")
endif()

# locations of libcmaes and NLOpt
include_directories("/usr/local/include")
link_directories("/usr/local/lib")

include_directories("/home/danesh/argos-sferes/src/ite") # location of local includes

add_definitions(-DUSE_NLOPT)
add_definitions(-DUSE_LIBCMAES)

set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -O3 -g -march=native" CACHE STRING "" FORCE)
message("CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS})

add_executable(ite_swarms ite_swarms.cpp)

target_link_libraries(ite_swarms
                        ${Boost_LIBRARIES}
                        ${TBB_LIBRARIES})
