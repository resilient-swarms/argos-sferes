#!/bin/bash


for dim in 3 10 ; do
    cd ~/argos-sferes
    mkdir build_${dim}D
    cd build_${dim}D
    if [ $dim -gt 3 ]
    then
    cmake -DNN_INPUT_TYPE=1 -DBO=ON -DRECORD_FITNESS=ON -DCVT_USAGE=ON -DBD=${dim} -DDEFINE_PRINT=ON ..
    else
    cmake -DNN_INPUT_TYPE=1 -DBO=ON -DCMAKE_BUILD_TYPE=Debug -DRECORD_FITNESS=ON -DBD=${dim}  -DDEFINE_PRINT=ON ..
    fi
   make -j 2


done 
