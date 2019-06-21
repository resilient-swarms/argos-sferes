cmake_minimum_required(VERSION 2.8.12)
project(argos_sferes)

set(SFERES ${PROJECT_SOURCE_DIR}/sferes2) # location of sferes2 - where https://github.com/sferes2/sferes2 has been cloned
include_directories(${SFERES})

set(THYMIO $ENV{THYMIO}) # location of Thymio plugin installation -- include and libs
IF( NOT THYMIO )
    MESSAGE( FATAL_ERROR "Please point the environment variable THYMIO to the location of the Thymio plugin installation")
ELSE()
    include_directories(${THYMIO}/include)
    link_directories(${THYMIO}/lib/argos3) 
ENDIF()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the ARGoS package
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(CMAKE_MODULE_PATH ${ARGOS_PREFIX}/share/argos3/cmake)

# Set the path where CMake will find additional scripts
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
message("CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH}) 

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
find_package(TBB) #COMPONENTS tbbmalloc tbbmalloc_proxy 
if(TBB_FOUND)
  include_directories(${TBB_INCLUDE_DIRS})
  message("TBB headers found in: " ${TBB_INCLUDE_DIRS}) 
  message("TBB libs: " ${TBB_LIBRARIES}) 
else(NOT TBB_FOUND)
  message("Unable to find TBB. Will not be able to run sferes eval::Parallel")
  add_definitions( -DNO_PARALLEL )
  set(TBB_INCLUDE_DIRS "")
  set(TBB_LIBRARIES "")
endif()

# Eigen3
SET(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
IF(NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ELSE()
    include_directories("${EIGEN3_INCLUDE_DIR}")
    add_definitions( -DEIGEN3_ENABLED )
ENDIF()


#link_directories(/usr/lib) ## careless -- which libs are we trying to link with?

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
add_definitions( -DNDEBUG ) # non-debug mode
add_definitions( -DVERSION="\(const char*\)\\"f731bfb04a48a475dfedbfdb180d77054498557f\\"" ) #value not important. added to prevent compilation error in sferes/run.hpp


#include_directories(/usr/include)  ## careless -- exactly which includes?

# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
include(ARGoSCheckQTOpenGL)

# Find the GSL package
find_package(GSL REQUIRED)
include_directories(${GSL_INCLUDE_DIR})

# Find Lua
find_package(Lua52)
IF(NOT LUA52_FOUND)
    MESSAGE( FATAL_ERROR "Please point the environment variable LUA_DIR to your Lua installation -- include and lib.")
ELSE()
   include_directories(${LUA_INCLUDE_DIR})
ENDIF()
#SET(LUA_DIR "$ENV{LUA_DIR}")
#IF(NOT LUA_DIR )
#    MESSAGE( FATAL_ERROR "Please point the environment variable LUA_DIR to your Lua installation -- include and lib.")
#ELSE()
#    include_directories("${LUA_DIR}/include")
#    link_directories("${LUA_DIR}/lib")
#ENDIF()

# Set ARGoS include dir
include_directories(${ARGOS_INCLUDE_DIRS})

# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS} ${THYMIO}/lib/argos3 ${PROJECT_SOURCE_DIR}/lib)


option(BASELINE "baseline behaviours vs evolution" OFF)
option(RECORD_FITNESS "print some variables" OFF)
option(DEFINE_PRINT "print some variables" OFF)
option(BD "behaviour description size" OFF)
option(CVT_USAGE "Centroidal Voronoi Tessellation MAP-Elites" OFF)
option(PERTURB "Perturbations" OFF)
option(BO "Bayesian Optimisation" OFF)




# process baseline option and others, to define controller, loopfunction and executable

if(BASELINE)
    message("will use baseline behaviours rather than evolution")
    add_definitions(-DBASELINEBEHAVS)  
    
    set(CONTROL "baseline-behavs")
    add_library(baseline-behavs SHARED
        src/baseline-behavs/baseline-behavs.h
        src/baseline-behavs/baseline-behavs.cpp
        src/baseline-behavs/behavior.h
        src/baseline-behavs/behavior.cpp
        src/baseline-behavs/dispersebehavior.h
        src/baseline-behavs/dispersebehavior.cpp
        src/baseline-behavs/randomwalkbehavior.h
        src/baseline-behavs/randomwalkbehavior.cpp
        src/baseline-behavs/homingtofoodbeaconbehavior.h
        src/baseline-behavs/homingtofoodbeaconbehavior.cpp
        src/baseline-behavs/aggregatebehavior.h
        src/baseline-behavs/aggregatebehavior.cpp
        src/baseline-behavs/coveragebehavior.h
        src/baseline-behavs/coveragebehavior.cpp
        src/baseline-behavs/bordercoveragebehavior.h
        src/baseline-behavs/bordercoveragebehavior.cpp
        src/baseline-behavs/flockingbehavior.h
        src/baseline-behavs/flockingbehavior.cpp
        src/baseline-behavs/circlebehavior.h
        src/baseline-behavs/circlebehavior.cpp
      )
     set(LOOPFUN baseline-behavs-loopfunc)
    set(baseline-behavs-loopfunc-SOURCES src/baseline-behavs/baseline-behavs-loopfunc.cpp)
    if(ARGOS_COMPILE_QTOPENGL)
      set(baseline-behavs-loopfunc-SOURCES
          ${baseline-behavs-loopfunc-SOURCES}
          src/core/robotid_vis.cpp)
    endif(ARGOS_COMPILE_QTOPENGL)

    add_library(baseline-behavs-loopfunc SHARED
        ${baseline-behavs-loopfunc-SOURCES})

    message("LF SOURCES is "${baseline-behavs-loopfunc-SOURCES})

    if(ARGOS_COMPILE_QTOPENGL)
        target_link_libraries(baseline-behavs-loopfunc argos3plugin_simulator_qtopengl)
    endif(ARGOS_COMPILE_QTOPENGL)

    set(EXECUTABLE baseline_behaviour)  
else(NOT BASELINE)
    message("will use evolution")
    message("Using ${BD}D behavioural characterisation")

    if(NOT ${BD})
        set(BD 2)
    endif()
    add_definitions(-DBEHAV_DIM=${BD})

    if(CVT_USAGE)
      set(CVT_USE cvt)
      add_definitions(-DCVT)
      message("Using Centroidal Voronoi Tesselation")
    else(NOT CVT_USAGE)
        if(NOT ${BD} EQUAL 1 AND NOT ${BD} EQUAL 2 AND NOT ${BD} EQUAL 3 AND NOT ${BD} EQUAL 6)
            message(FATAL_ERROR "When not using CVT, need to use 1, 2,3 or 6 behavioural dimensions")
        endif()
    endif()


    set(CONTROL "nn_controller")

    set(tag ${CVT_USE}${BD}D)


# process BO option
if(BO)
    message("Bayesian Optimisation is defined")
    add_definitions(-DBAYESIAN_OPT)   
    set(tag BO${CVT_USE}${BD}D)
else(NOT BO)
    message("Bayesian Optimisation is NOT defined")
endif()
    
    add_definitions(-DTAG="${tag}")


if(ARGOS_COMPILE_QTOPENGL)
    add_library(evolution_loopfunctions${tag} SHARED
      src/evolution/evol_loop_functions.h
      src/evolution/evol_loop_functions.cpp
        src/core/robotid_vis.h
       src/core/robotid_vis.cpp
    )
elseif(NOT ARGOS_COMPILE_QTOPENGL)
        add_library(evolution_loopfunctions${tag} SHARED
      src/evolution/evol_loop_functions.h
      src/evolution/evol_loop_functions.cpp
    )
endif()


    add_library(descriptors${tag} SHARED
 src/evolution/descriptors.h
        src/evolution/descriptors.cpp
)

    # Descend into the controllers directory
    add_library(nn_controller SHARED
    src/evolution/nn_controller.h
    src/evolution/nn_controller.cpp
    )
    

    set(LOOPFUN evolution_loopfunctions${tag})

    set(EXECUTABLE behaviour_evol${tag})
endif()




# general options

if(RECORD_FITNESS)
    message("will record fitness after each evaluation (useful for tests)")
    add_definitions(-DRECORD_FIT)    
else(NOT RECORD_FITNESS)
    message("will NOT record fitness after each evaluation (useful for simulation)")
endif()


if(DEFINE_PRINT)
    message("print is defined")
    add_definitions(-DPRINTING)    
else(NOT DEFINE_PRINT)
    message("print is NOT defined")
endif()







# core libraries


add_library(statistics SHARED
   src/core/statistics.h
   src/core/statistics.cpp
)

add_library(base_loopfunctions SHARED
  src/core/base_loop_functions.h
  src/core/base_loop_functions.cpp
  src/core/fitness_functions.h # added as error argos3: symbol lookup error: [path removed]/libbase_loopfunctions.so: undefined symbol: _ZN11AggregationC1Ev
  src/core/fitness_functions.cpp
  src/core/statistics.h # added as error argos3: symbol lookup error: [path removed] libbase_loopfunctions.so: undefined symbol: _ZN8StatFuns12get_avg_distESt6vectorIN5argos8CVector3ESaIS2_EES2_
  src/core/statistics.cpp
)

add_library(base_controller SHARED
  src/core/base_controller.h
  src/core/base_controller.cpp
)


add_library(arena_utils${tag} SHARED
  src/core/arena_utils.h
  src/core/arena_utils.cpp
)

add_library(fitness_functions${tag} SHARED
  src/core/fitness_functions.h
  src/core/fitness_functions.cpp
)




add_library(communication SHARED
  src/core/sensingandcommunication.h
  src/core/sensingandcommunication.cpp
)





# add executable

add_executable(${EXECUTABLE} src/main.cpp)

# link the controller
      target_link_libraries(${CONTROL}
      base_controller
      communication
      argos3core_simulator
      #argos3plugin_simulator_actuators
      argos3plugin_simulator_thymio
      argos3plugin_simulator_genericrobot
      #argos3plugin_simulator_sensors
      ${LUA_LIBRARIES}
      ${Boost_LIBRARIES}
      ${TBB_LIBRARIES})

# link the loop functions
    target_link_libraries(${LOOPFUN}
      base_loopfunctions
      ${CONTROL}
      base_controller
      argos3core_simulator
      #argos3plugin_simulator_actuators
      argos3plugin_simulator_dynamics2d
      argos3plugin_simulator_entities
      argos3plugin_simulator_thymio
      argos3plugin_simulator_genericrobot
      argos3plugin_simulator_media
      #argos3plugin_simulator_sensors
      ${LUA_LIBRARIES}
      ${Boost_LIBRARIES}
${TBB_LIBRARIES})



# link the executable based on BASELINE setting (only difference so far is descriptor)

if(BASELINE)      
      target_link_libraries(${EXECUTABLE}
      ${LOOPFUN}
      base_loopfunctions
      ${CONTROL}
      fitness_functions${tag}
      arena_utils${tag}
      statistics
      argos3core_simulator 
      ${LUA_LIBRARIES}
      ${Boost_LIBRARIES} 
${TBB_LIBRARIES})
   
else(NOT BASELINE)
   target_link_libraries(${EXECUTABLE}
      ${LOOPFUN}
      base_loopfunctions
      ${CONTROL}
      descriptors${tag}
      fitness_functions${tag}
      arena_utils${tag}
      statistics
      argos3core_simulator 
      ${LUA_LIBRARIES}
      ${Boost_LIBRARIES} 
${TBB_LIBRARIES})

endif()
   






  
