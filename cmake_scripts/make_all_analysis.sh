#!/bin/bash


# compile envir_parallel stuff
for dim in 6; do   # 10 14 21 400 
    cd ~/argos-sferes
    mkdir build_${dim}DANA
    cd build_${dim}DANA
    if [ $dim -gt 10 ]
    then
    cmake -DNN_INPUT_TYPE=0 -DCVT_USAGE=ON -DBD=${dim} -DANALYSIS=ON ..
    else
    cmake -DNN_INPUT_TYPE=0 -DBD=${dim} -DANALYSIS=ON ..
    fi
   make -j 8


done 



#compile behaviour_evol stuff
for dim in 1024 3 10; do   # 10 14 21 400 
    cd ~/argos-sferes
    mkdir build_${dim}DANA
    cd build_${dim}DANA
    if [ $dim -gt 3 ]
    then
    cmake -DNN_INPUT_TYPE=0 -DANALYSIS=ON -DCVT_USAGE=ON -DBD=${dim}  ..
    else
    cmake -DNN_INPUT_TYPE=0 -DANALYSIS=ON -DBD=${dim} ..
    fi
   make -j 8


done 
