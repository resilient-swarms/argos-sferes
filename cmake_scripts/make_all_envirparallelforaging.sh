#!/bin/bash


for dim in 6; do   # 10 14 21 400 
    cd ~/argos-sferes
    mkdir build_${dim}D
    cd build_${dim}D
    if [ $dim -gt 10 ]
    then
    cmake -DNN_INPUT_TYPE=1 -DARGOS_PAR=2 -DCVT_USAGE=ON -DBD=${dim}  ..
    else
    cmake -DNN_INPUT_TYPE=1 -DARGOS_PAR=2 -DBD=${dim} ..
    fi
   make -j 8


done 
