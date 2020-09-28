#!/bin/bash


declare -A acq_funs
acq_funs[0]="UCB"
acq_funs[1]="UCB_ID"
for alpha in 0.05 0.25 0.50 1; do
    for l in 0.05 0.1 0.2 0.4 1; do
        for acq in ${!acq_funs[@]}; do
			bash cmake_scripts/make_all_heterosim.sh $acq 0 $alpha $l
			acq_string=${acq_funs[${acq}]}
			export EXPERIMENT_TAG="alpha${alpha}_l${l}_${acq_string}_M52VarNoise"
			echo "start runs with experiment tag: ${EXPERIMENT_TAG}"
			bash launch_foraging_BO.sh /scratch/dmb1m19 BO_single_IDprior ${EXPERIMENT_TAG}
		done
    done
done

