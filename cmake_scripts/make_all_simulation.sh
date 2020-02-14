#!/bin/bash


for dim in 3 ; do   # 10 14 21 400 
    cd ~/argos-sferes
    mkdir build_${dim}D
    cd build_${dim}D
    rm -rf *
    if [ $dim -gt 3 ]; then
    
      cmake -DNN_INPUT_TYPE=1 -DCMAKE_BUILD_TYPE=Debug -DCVT_USAGE=ON -DBD=${dim}  ..
    else
      cmake -DNN_INPUT_TYPE=0 -DCMAKE_BUILD_TYPE=Debug -DBD=${dim} ..
    fi
   make -j 8


done 
