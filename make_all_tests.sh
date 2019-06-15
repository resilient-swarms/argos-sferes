#!/bin/bash


for dim in 2 10 14 21 400; do
    cd ~/argos-sferes
    mkdir build_${dim}D
    cd build_${dim}D
    if [ $dim -gt 3 ]
    then
    cmake -DCMAKE_BUILD_TYPE=Debug -DCVT_USAGE=ON -DBD=${dim} -DRECORD_FITNESS=ON -DDEFINE_PRINT=OFF ..
    else
    cmake -DCMAKE_BUILD_TYPE=Debug -DBD=${dim} -DRECORD_FITNESS=ON -DDEFINE_PRINT=OFF ..
    fi
   make 


done 
