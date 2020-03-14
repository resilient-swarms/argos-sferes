#!/bin/bash


for dim in 3 10 ; do
    mkdir build_${dim}D
    cd build_${dim}D
    if [ $dim -gt 3 ]
    then
    cmake -DBO=ON -DRECORD_FITNESS=ON -DCVT_USAGE=ON -DBD=${dim} -DDEFINE_PRINT=OFF ..
    else
    cmake -DBO=ON -DCMAKE_BUILD_TYPE=Debug -DRECORD_FITNESS=ON -DBD=${dim}  -DDEFINE_PRINT=OFF ..
    fi
   make -j 8
   cd ..


done 
