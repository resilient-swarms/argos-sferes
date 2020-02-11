#!/bin/bash


mkdir build_ite
cd build_ite
cmake -DBD=3 -DANALYSIS=OFF -DNN_INPUT_TYPE=0 -DDEFINE_PRINT=OFF -DBO=ON -DRECORD_FITNESS=ON ..
make -j 8
