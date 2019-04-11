#!/bin/bash -e


data=experiments/data
maxnumjobs=8

# Create a data diretory
mkdir -p $data



for FitfunType in MeanSpeed; do
    for DescriptorType in history; do
        mkdir -p $data/${FitfunType}/${DescriptorType}
        for Replicates in $(seq 1 5); do
                              trials="TRIALS"
            
            # Take template.argos and make an .argos file for this experiment
            SUFFIX=${Replicates}${Replicates}${Replicates}
            Outfolder=$data/${FitfunType}/${DescriptorType}
            sed -e "s|TRIALS|15|" \
                -e "s|ROBOTS|1|"                    \
                -e "s|SEED|${Replicates}${Replicates}${Replicates}|"                    \
                -e "s|FITFUN_TYPE|${FitfunType}|"                   \
                -e "s|DESCRIPTOR_TYPE|${DescriptorType}|"                  \
                -e "s|OUTPUTFOLDER|${Outfolder}|" \
                experiments/experiment_template.argos                       \
                > ${Outfolder}/exp_${SUFFIX}.argos
            # Call ARGoS
            #parallel --semaphore -j${maxnumjobs} argos3 -c $data/${FitfunType}/${DescriptorType}/exp_${SUFFIX}.argos &
        done
    done
done
