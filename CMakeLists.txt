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
find_package(Boost 1.66 COMPONENTS system serialization graph program_options filesystem  REQUIRED)
# system unit_test_framework thread REQUIRED)
if(Boost_FOUND)
    link_directories(${Boost_LIBRARY_DIRS})
    include_directories(${Boost_INCLUDE_DIRS})
    message("Boost libs found in: " ${Boost_LIBRARY_DIRS})
    message("Boost headers found in: " ${Boost_INCLUDE_DIRS})
else(NOT Boost_FOUND)
	link_directories(${BOOSTROOT}/lib)
	include_directories(${BOOSTROOT}/include)
	#message(FATAL_ERROR "Unable to continue without BOOST")
endif()



	#Find TBB libs -- script from https://github.com/justusc/FindTBB
	find_package(TBB COMPONENTS tbbmalloc tbbmalloc_proxy) 
	if(TBB_FOUND)
                message("TBB headers found in: " ${TBB_INCLUDE_DIRS})
    		message("TBB libs: " ${TBB_LIBRARIES})
               add_definitions( -DNO_PARALLEL )
	       	message("Parallelization is to be disabled if using argos visualization")
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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -lboost_system -fPIC")
add_definitions( -DNDEBUG ) # non-debug mode
add_definitions( -DVERSION="\(const char*\)\\"f731bfb04a48a475dfedbfdb180d77054498557f\\"" ) #value not important. added to prevent compilation error in sferes/run.hpp

# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
include(ARGoSCheckQTOpenGL)

# Find the GSL package
find_package(GSL REQUIRED)
include_directories(${GSL_INCLUDE_DIR})

# Find Lua
find_package(Lua53)
IF(NOT LUA53_FOUND)
    MESSAGE("Please point the environment variable LUA_DIR to your Lua installation -- include and lib.")
    MESSAGE("Will base include of lua on your environment variable LUA_DIR")
    include_directories($ENV{LUA_DIR}/include)
ELSE()
    include_directories(${LUA_INCLUDE_DIR})
ENDIF()

# Set ARGoS include dir
include_directories(${ARGOS_INCLUDE_DIRS})

# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS} ${THYMIO}/lib/argos3 ${PROJECT_SOURCE_DIR}/lib)


option(BASELINE "baseline behaviours vs evolution" OFF)
option(RECORD_FITNESS "print some variables" OFF)
option(DEFINE_PRINT "print some variables" OFF)
option(BD "behaviour description size" OFF)
option(CVT_USAGE "Centroidal Voronoi Tessellation MAP-Elites" OFF)
option(ANALYSIS "Perturbations" OFF)
option(BO "Bayesian Optimisation" OFF)
option(ARGOS_PAR "Parallelisation" OFF)
option(CAFFE "caffe neural networks" OFF)
option(NN_INPUT_TYPE "control RAB" OFF)

#add_definitions(-DCOLLISION_STOP)  


    
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lc -lrt") # use something similar if using file descriptors



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
        src/baseline-behavs/obstacleavoidancebehavior.h
        src/baseline-behavs/obstacleavoidancebehavior.cpp
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
        src/baseline-behavs/src_chain_behavior.h
        src/baseline-behavs/src_chain_behavior.cpp
        src/baseline-behavs/link_chain_behavior.h
        src/baseline-behavs/link_chain_behavior.cpp
        src/baseline-behavs/dest_chain_behavior.h
        src/baseline-behavs/dest_chain_behavior.cpp
        src/baseline-behavs/src_chain_behavior1.h
        src/baseline-behavs/src_chain_behavior1.cpp
        src/baseline-behavs/link_chain_behavior1.h
        src/baseline-behavs/link_chain_behavior1.cpp
        src/baseline-behavs/dest_chain_behavior1.h
        src/baseline-behavs/dest_chain_behavior1.cpp
        src/baseline-behavs/flockingbehavior.h
        src/baseline-behavs/flockingbehavior.cpp
        src/baseline-behavs/circlebehavior.h
        src/baseline-behavs/circlebehavior.cpp)
     set(LOOPFUN baseline-behavs-loopfunctions)
     set(baseline-behavs-loopfunc-SOURCES src/baseline-behavs/baseline-behavs-loopfunc.cpp)
     if(ARGOS_COMPILE_QTOPENGL)
        set(baseline-behavs-loopfunc-SOURCES
            ${baseline-behavs-loopfunc-SOURCES}
            src/core/robotid_vis.cpp)
     endif(ARGOS_COMPILE_QTOPENGL)

     add_library(baseline-behavs-loopfunctions SHARED
        ${baseline-behavs-loopfunc-SOURCES})

    message("LF SOURCES is " ${baseline-behavs-loopfunc-SOURCES})

    if(ARGOS_COMPILE_QTOPENGL)
        target_link_libraries(baseline-behavs-loopfunctions argos3plugin_simulator_qtopengl)
    endif(ARGOS_COMPILE_QTOPENGL)

    set(EXECUTABLE baseline_behaviour) 
    set(EXEC_FILE baselinebehavs.cpp) 
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
        if(NOT ${BD} EQUAL 1 AND NOT ${BD} EQUAL 2 AND NOT ${BD} EQUAL 3 AND NOT ${BD} EQUAL 6 AND NOT ${BD} EQUAL 7)
            message(FATAL_ERROR "When not using CVT, need to use 1, 2,3, 6 or 7 behavioural dimensions")
        endif()
    endif()

    

    set(tag ${CVT_USE}${BD}D)
    if(ANALYSIS)
        set(tag ${tag}ANA)
    endif()


    # process NN_INPUT_TYPE
    if(${NN_INPUT_TYPE} EQUAL 0)
        set(CONTROL "nn_controller")
        message("using proximity and RAB (NN_INPUT_TYPE=0)")
        set(exec_prefix "behaviour_evol")
            # Descend into the controllers directory
        add_library(nn_controller SHARED
            src/evolution/nn_controller.h
            src/evolution/nn_controller.cpp)

        
        add_definitions(-DNN_DIM_TYPE=0)

    elseif(${NN_INPUT_TYPE} EQUAL 1)
        set(CONTROL "foraging_nn_controller")
        message("using foraging-nn controller: proximity and ground (NN_INPUT_TYPE=1)")
        set(tag ${tag}REAL)
        set(exec_prefix "behaviour_evol")
        add_definitions(-DNN_DIM_TYPE=1)
            # Descend into the controllers directory
        add_library(foraging_nn_controller SHARED
            src/evolution/foraging_nn_controller.h
            src/evolution/foraging_nn_controller.cpp)
    elseif(${NN_INPUT_TYPE} EQUAL 2)
        set(CONTROL "nn_controller")
        message("using rab-control (NN_INPUT_TYPE=2)")
        set(exec_prefix "behaviour_evol")
        add_definitions(-DNN_DIM_TYPE=2)
        add_library(nn_controller SHARED
            src/evolution/nn_controller.h
            src/evolution/nn_controller.cpp)
        add_library(station_controller SHARED
        src/core/station_controller.cpp
         src/core/station_controller.h)
        list(APPEND CONTROL "station_controller")
    else()
         MESSAGE( FATAL_ERROR "Please choose an input type for the neural network (NN_DIM_TYPE={0,1,2})")
    endif()

    # process BO option
    if(BO)
        message("Bayesian Optimisation is defined")

        #######################################################################
        # For ite-swarms
        set(LIMBO_PATH ${PROJECT_SOURCE_DIR}/limbo/src) # location of limbo - where https://github.com/resibots/limbo.git has been cloned
        include_directories(${LIMBO_PATH})

        #SET(NLOPT_INCLUDE_DIR "$ENV{NLOPT_INCLUDE_DIR}")
        #SET(NLOPT_LIB_DIR "$ENV{NLOPT_LIB_DIR}")
        #IF(NOT NLOPT_INCLUDE_DIR OR NOT NLOPT_LIB_DIR)
        #    MESSAGE( WARNING "NLOpt not found. Please point the environment variables NLOPT_INCLUDE_DIR and NLOPT_LIB_DIR to your NLopt installation.")
        #ELSE()
        #    include_directories("${NLOPT_INCLUDE_DIR}")
        #    link_directories("${NLOPT_LIB_DIR}")
        #   add_definitions(-DUSE_NLOPT)
        #ENDIF()

        SET(CMAES_INCLUDE_DIR "$ENV{CMAES_INCLUDE_DIR}")
        SET(CMAES_LIB_DIR "$ENV{CMAES_LIB_DIR}")
        IF(NOT CMAES_INCLUDE_DIR OR NOT CMAES_LIB_DIR)
            MESSAGE( WARNING "CMAES not found. Please point the environment variables CMAES_INCLUDE_DIR and CMAES_LIB_DIR to your CMAES installation.")
        ELSE()
            include_directories("${CMAES_INCLUDE_DIR}")
            link_directories("${CMAES_LIB_DIR}")
            add_definitions(-DUSE_LIBCMAES)
        ENDIF()

        include_directories(${PROJECT_SOURCE_DIR}/src/ite) # location of ite-swarm includes

        message("CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS})
        set(TMP_CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
        set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -O3 -g -march=native" CACHE STRING "" FORCE)  
        set(tag BO${tag})
        add_executable(ite_swarms_${tag}
            src/ite/ite_swarms.cpp)
        target_link_libraries(ite_swarms_${tag}
            ${Boost_LIBRARIES}
            ${TBB_LIBRARIES})

        add_executable(ite_swarms_VE_${tag}
            src/ite/ite_swarms_VE.cpp)
        target_link_libraries(ite_swarms_VE_${tag}
            ${Boost_LIBRARIES}
            ${TBB_LIBRARIES})

        add_executable(tuning${tag}
            src/ite/tuning.cpp)
        message("CMAES LIB DIR" ${CMAES_LIB_DIR})
        target_link_libraries(tuning${tag}
          ${Boost_LIBRARIES}
          ${TBB_LIBRARIES}
          ${CMAES_LIB_DIR}/libcmaes.so
          )

        add_executable(tuning_VE_${tag}
            src/ite/tuning_VE.cpp)
        message("CMAES LIB DIR" ${CMAES_LIB_DIR})
        target_link_libraries(tuning_VE_${tag}
          ${Boost_LIBRARIES}
          ${TBB_LIBRARIES}
          ${CMAES_LIB_DIR}/libcmaes.so
          )

        set(CMAKE_CXX_FLAGS ${TMP_CMAKE_CXX_FLAGS} CACHE STRING "" FORCE)
        #set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -llibcmaes -lm -lglog -lgflags ")
        message(FLAGS ${CMAKE_CXX_FLAGS})
        message("${PROJECT_SOURCE_DIR} " ${PROJECT_SOURCE_DIR})
        #######################################################################

        # For the argos simulator called by ite-swarms Eval function
        add_definitions(-DBAYESIAN_OPT)
        set(RECORD_FITNESS ON)
        set(EXECUTABLE ${tag})
        set(EXEC_FILE BO.cpp)
    else(NOT BO)
        message("Bayesian Optimisation is NOT defined")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -g -march=native " )  
	    message("CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS})

        if(${ARGOS_PAR} EQUAL 1)
            message("using parallel (1)")
            add_definitions(-DARGOS_PARALLEL) 
            set(EXECUTABLE ${exec_prefix}${tag})
            set(EXEC_FILE ${exec_prefix}.cpp)
        elseif(${ARGOS_PAR} EQUAL 2)
            message("using parallel environments (2)")  
            add_definitions(-DARGOS_PARALLELENVIR) 
            set(EXECUTABLE envir_evol${tag})
            set(EXEC_FILE envir_evol.cpp)
        else()
            message("using serial (0)")
            set(EXECUTABLE ${exec_prefix}${tag})
            set(EXEC_FILE ${exec_prefix}.cpp)
        endif()
    endif()
    
    add_definitions(-DTAG="${tag}")
    message("TAG" ${tag})
    if(ARGOS_COMPILE_QTOPENGL)
        message("QT")
        if(${NN_INPUT_TYPE} EQUAL 1)
            message("Foraging")
            add_library(foraging_loopfunctions${tag} SHARED
            src/evolution/params.h
            src/evolution/virtual_energy.h
            src/evolution/virtual_energy.cpp
            src/evolution/foraging_loop_functions.h
            src/evolution/foraging_loop_functions.cpp
             src/core/robotid_vis.h
            src/core/robotid_vis.cpp)
	    
            #add_library(foraging_vis SHARED
#
	    #src/evolution/foraging_vis.h
           # src/evolution/foraging_vis.cpp
	   # )
	    
            set(LOOPFUN foraging_loopfunctions${tag})
        elseif(NOT ${NN_INPUT_TYPE} EQUAL 1)
            message("Evolution")
             add_library(evolution_loopfunctions${tag} SHARED
            src/evolution/params.h
            src/evolution/evol_loop_functions.h
            src/evolution/evol_loop_functions.cpp
            src/core/robotid_vis.h
            src/core/robotid_vis.cpp)
            set(LOOPFUN evolution_loopfunctions${tag})


        endif()
	
    elseif(NOT ARGOS_COMPILE_QTOPENGL)
    message("NO QT")
        if(${NN_INPUT_TYPE} EQUAL 1)
        message("Foraging")
            add_library(foraging_loopfunctions${tag} SHARED
            src/evolution/params.h
            src/evolution/foraging_loop_functions.h
            src/evolution/foraging_loop_functions.cpp
            src/evolution/virtual_energy.h
            src/evolution/virtual_energy.cpp
	        )
            set(LOOPFUN foraging_loopfunctions${tag})
        elseif(NOT ${NN_INPUT_TYPE} EQUAL 1)
        message("Evolution")
             add_library(evolution_loopfunctions${tag} SHARED
            src/evolution/params.h
            src/evolution/evol_loop_functions.h
            src/evolution/evol_loop_functions.cpp)
            set(LOOPFUN evolution_loopfunctions${tag})
        endif()
    endif()

    add_library(descriptors${tag} SHARED
        src/evolution/descriptors.h
        src/evolution/descriptors.cpp
    )



    
    
    
endif()

set(AdditionalLibs "")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -luuid") # somehow need to set this even when not using Caffe 
if(CAFFE)
    message("Caffe on")
    add_definitions(-DCAFFE_NETS)
    set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ~/caffe)

    #set(Caffe_LIBRARIES "/home/david/caffe/build/install/lib/libcaffe.so")
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lcaffe " ) 
    #include_directories(${CAFFE_INCLUDE_DIRS})

    add_definitions("-DCPU_ONLY=1") # added this one manually
    #add_definitions("-DUSE_OPENCV=1") # added this one manually
    find_package(GLog REQUIRED)


    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -luuid -lboost_system -lglog")


    #find_package(Caffe)  # check  ~/.cmake/packages/Caffe

    #message(STATUS "Caffe_DEFINITIONS")
    #message(STATUS ${Caffe_DEFINITIONS})

    #message(STATUS "Caffe_INCLUDE_DIRS")
    #message(STATUS ${Caffe_INCLUDE_DIRS})  #added this one manually

    #message(STATUS "Caffe_LIBRARIES")
    #message(STATUS ${Caffe_LIBRARIES})

    #include_directories($ENV{CAFFE_INCLUDE_DIRS})



    unset(Caffe_FOUND)

    ###Set the variable Caffe_DIR as the root of your caffe directory
    set(Caffe_DIR ~/caffe/build)

    find_path(Caffe_INCLUDE_DIRS NAMES caffe/caffe.hpp caffe/common.hpp caffe/net.hpp caffe/proto/caffe.pb.h caffe/util/io.hpp caffe/vision_layers.hpp
    HINTS
    ${Caffe_DIR}/include)

    find_library(Caffe_LIBRARIES NAMES caffe
    HINTS
    ${Caffe_DIR}/lib)

    message("lib_dirs:${Caffe_LIBRARIES}")

    if(Caffe_LIBRARIES AND Caffe_INCLUDE_DIRS)
    set(Caffe_FOUND 1)
    endif()
    include_directories(${Caffe_INCLUDE_DIRS})
    add_library(caffenets SHARED
                src/caffe_nets/caffe_net.h
                src/caffe_nets/caffe_net.cpp)

    #find_package(Caffe) #if you want  to avoid OpenCV error


    add_definitions(-DTAG="${tag}")

    # uncomment if you just want to check neural network           
    set(EXECUTABLE caffenetexec)
    set(EXEC_FILE caffe_nets/caffe_net.cpp)

    set(AdditionalLibs  ${Caffe_LIBRARIES} ${GLOG_LIBRARIES})
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

if(ANALYSIS)
    message("will perform analysis; changing executable to 'analysis'")
    message("make sure you are using 'analysis' descriptor")
    add_definitions(-DANALYSIS)
    set(EXECUTABLE analysis${tag})
    set(EXEC_FILE analysis.cpp)  


    # this one is used as a default, we don't need the evolution anyway
    # assumes you have compiled cvt10D
endif()




# core libraries

add_library(statistics SHARED
    src/core/statistics.h
    src/core/statistics.cpp)

add_library(base_loopfunctions${tag} SHARED
    src/evolution/base_evol_loop_functions.h
    src/evolution/base_evol_loop_functions.cpp
    src/core/base_loop_functions.h
    src/core/base_loop_functions.cpp
    src/core/fitness_functions.h # added as error argos3: symbol lookup error: [path removed]/libbase_loopfunctions.so: undefined symbol: _ZN11AggregationC1Ev
    src/core/fitness_functions.cpp
    src/core/statistics.h # added as error argos3: symbol lookup error: [path removed] libbase_loopfunctions.so: undefined symbol: _ZN8StatFuns12get_avg_distESt6vectorIN5argos8CVector3ESaIS2_EES2_
    src/core/statistics.cpp
    src/core/environment_generator.h
    src/core/environment_generator.cpp

)

add_library(base_controller SHARED
    src/core/base_controller.h
    src/core/base_controller.cpp)


add_library(arena_utils${tag} SHARED
    src/core/arena_utils.h
    src/core/arena_utils.cpp)

add_library(fitness_functions${tag} SHARED
    src/core/fitness_functions.h
    src/core/fitness_functions.cpp)

add_library(communication SHARED
    src/core/sensingandcommunication.h
    src/core/sensingandcommunication.cpp)

add_executable(${EXECUTABLE} src/${EXEC_FILE})

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
    ${TBB_LIBRARIES}
    ${Caffe_LIBRARIES}
)

# link the loop functions
target_link_libraries(${LOOPFUN}

    base_loopfunctions${tag}
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
    ${TBB_LIBRARIES}
    ${AdditionalLibs}
)

# link the executable based on BASELINE setting (only difference so far is descriptor)
if(BASELINE)      
    target_link_libraries(${EXECUTABLE}
        ${LOOPFUN}
        base_loopfunctions${tag}
        ${CONTROL}
        fitness_functions${tag}
        arena_utils${tag}
        statistics
        argos3core_simulator
        ${LUA_LIBRARIES}
        ${Boost_LIBRARIES}
        ${TBB_LIBRARIES}
        ${AdditionalLibs}
        
)
else(NOT BASELINE)
   target_link_libraries(${EXECUTABLE}
        ${LOOPFUN}
        base_loopfunctions${tag}
        ${CONTROL}
        descriptors${tag}
        fitness_functions${tag}
        arena_utils${tag}
        statistics
        argos3core_simulator
        ${LUA_LIBRARIES}
        ${Boost_LIBRARIES}
        ${TBB_LIBRARIES}
        ${AdditionalLibs}
)
endif()

