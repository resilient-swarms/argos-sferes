#!/bin/bash


for dim in 3 10 576; do   # 10 14 21 400 
    cd ~/argos-sferes
    mkdir build_${dim}D
    cd build_${dim}D
    if [ $dim -gt 3 ]
    then
    cmake -DARGOS_PAR=1 -DCVT_USAGE=ON -DBD=${dim}  ..
    else
    cmake -DARGOS_PAR=1 -DBD=${dim} ..
    fi
   make -j 8


done 
