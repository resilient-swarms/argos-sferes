#!/bin/bash


for dim in 6 7; do   # 10 14 21 400 
    mkdir build_${dim}D
    cd build_${dim}D
    if [ $dim -gt 10 ]
    then
    cmake -DARGOS_PAR=2 -DNN_INPUT_TYPE=0 -DCVT_USAGE=ON -DBD=${dim}  ..
    else
    cmake -DARGOS_PAR=2 -DNN_INPUT_TYPE=0 -DBD=${dim} ..
    fi
   make -j 8
   cd ..


done 
