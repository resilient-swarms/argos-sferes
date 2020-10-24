#!/bin/bash

for alpha in 0.93; do
    for l in 0.12; do
        for kern in 0; do
            for acq in 0; do
                bash cmake_scripts/make_all_heterosim.sh "$acq" "$kern" "$alpha" "$l"  
            done
        done
    done
done
