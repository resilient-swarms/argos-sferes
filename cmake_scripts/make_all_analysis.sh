#!/bin/bash


#compile behaviour_evol stuff
for dim in 3; do   # 10 14 21 400 
    cd ~/argos-sferes
    mkdir build_${dim}DANA
    cd build_${dim}DANA
    if [ $dim -gt 6 ]
    then
    cmake -DLARGE=ON -DRECORD_FITNESS=ON -DNN_INPUT_TYPE=1 -DANALYSIS=ON -DCVT_USAGE=ON -DBD=${dim}  ..
    else
    cmake -DLARGE=ON -DRECORD_FITNESS=ON -DNN_INPUT_TYPE=1 -DANALYSIS=ON -DBD=${dim} ..
    fi
   make 


done 
