#!/bin/bash

for alpha in 0.05 0.25 0.50 1; do
    for l in 0.05 0.1 0.2 0.4 1; do
        for kern in 0; do
            for acq in 0 1; do
                bash cmake_scripts/make_all_heterosim.sh $acq $kern $alpha $l  
            done
        done
    done
done