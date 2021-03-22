#!/bin/bash


cd ~/argos-sferes

mkdir buildbaseline && cd buildbaseline

cmake -DCMAKE_BUILD_TYPE=Debug -DRECORD_FITNESS=ON -DBASELINE=ON -DNN_INPUT_TYPE=0 ..

make
