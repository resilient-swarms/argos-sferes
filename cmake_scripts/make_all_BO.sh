#!/bin/bash




for dim in 3 10 ; do
    mkdir build_${dim}D
    cd build_${dim}D
    if [ $dim -gt 3 ]
    then
    cmake -DLARGE=ON -DNN_INPUT_TYPE=1 -DBO=ON -DRECORD_FITNESS=ON -DCVT_USAGE=ON -DBD=${dim} ..
    else
    cmake -DLARGE=ON -DNN_INPUT_TYPE=1 -DBO=ON -DCMAKE_BUILD_TYPE=Debug -DRECORD_FITNESS=ON -DBD=${dim} ..
    fi
<<<<<<< HEAD
   make -j 2
=======
   make -j 8
   cd ..
>>>>>>> 2e0e10fa06235d5891c0ac67b49d0ab44eb89ac5


done 
