#!/bin/bash


# compile envir_parallel stuff
for dim in 6 7; do   # 10 14 21 400 
    cd ~/argos-sferes
    mkdir build_${dim}D
    cd build_${dim}D
    if [ $dim -gt 10 ]
    then
    cmake -DCVT_USAGE=ON -DBD=${dim} -DANALYSIS=ON ..
    else
    cmake -DBD=${dim} -DANALYSIS=ON ..
    fi
   make -j 8


done 



#compile behaviour_evol stuff
for dim in 10 576; do   # 10 14 21 400 
    cd ~/argos-sferes
    mkdir build_${dim}D
    cd build_${dim}D
    if [ $dim -gt 3 ]
    then
    cmake -DANALYSIS=ON -DCVT_USAGE=ON -DBD=${dim}  ..
    
    cmake -DANALYSIS=ON -DBD=${dim} ..
    fi
   make -j 8


done 