#!/bin/bash


for dim in 1 2 10 14 21 400; do
    cd ~/argos-sferes
    mkdir build_${dim}D
    cd build_${dim}D
    if [ $dim -gt 3 ]
    then
    cmake -DCMAKE_BUILD_TYPE=Debug -DCVT_USAGE=ON -DBD=${dim}  ..
    else
    cmake -DCMAKE_BUILD_TYPE=Debug -DBD=${dim} ..
    fi
   make 


done 
