#!/bin/bash


cd ~/argos-sferes

mkdir buildbaseline && cd buildbaseline

cmake -DRECORD_FITNESS=ON -DBASELINE=ON ..

make
