#!/bin/bash




for dim in 3 10 ; do
    cd ~/argos-sferes
    mkdir build_${dim}D
    cd build_${dim}D
    if [ $dim -gt 3 ]
    then
    cmake -DHETEROGENEOUS=ON -DNN_INPUT_TYPE=1 -DBO=ON -DRECORD_FITNESS=OFF -DCVT_USAGE=ON -DBD=${dim} ..
    else
    cmake -DHETEROGENEOUS=ON -DNN_INPUT_TYPE=1 -DBO=ON -DCMAKE_BUILD_TYPE=Debug -DRECORD_FITNESS=OFF -DBD=${dim} ..
    fi
   make -j 2


done 
