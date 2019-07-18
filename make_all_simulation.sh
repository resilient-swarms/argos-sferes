#!/bin/bash


for dim in 10 576; do   # 10 14 21 400 
    cd ~/argos-sferes
    mkdir build_${dim}D
    cd build_${dim}D
    if [ $dim -gt 3 ]
    then
    cmake -DCVT_USAGE=ON -DBD=${dim}  ..
    else
    cmake -DBD=${dim} ..
    fi
   make -j 8


done 
