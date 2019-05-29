#!/bin/bash


for dim in 2 14 21 400; do
    cd ~/argos-sferes
    mkdir build_${dim}D
    cd build_${dim}D
    if [ $dim -gt 3 ]
    then
    cmake -DCVT_USAGE=ON -DBD=${dim} -DRECORD_FITNESS=OFF -DDEFINE_PRINT=OFF ..
    else
    cmake -DBD=${dim} -DRECORD_FITNESS=OFF -DDEFINE_PRINT=OFF ..
    fi
   make 


done 
