#!/bin/bash

datafolder=$1
if [ -z "${datafolder}" ]; then
	echo "Error: no datafolder given"
	exit 125
fi
declare -A BO_exps
acq_funs[0]="UCB_LOCAL2"
#acq_funs[1]="UCB_LOCAL"
#acq_funs[3]="UCB"
#acq_funs[1]="UCB_ID"

BO_exps["UCB_LOCAL2"]="BO_single_multi"
#BO_exps["UCB_LOCAL"]="BO_single_multi_record"
#BO_exps["UCB_ID"]="BO_single_IDprior"
for alpha in 0.93; do
    for l in  0.12; do
        for acq in ${!acq_funs[@]}; do
		        #bash cmake_scripts/make_all_heterosim.sh $acq 0 $alpha $l
			acq_string=${acq_funs[${acq}]}
			export EXPERIMENT_TAG="alpha${alpha}_l${l}_${acq_string}_M52VarNoise"
			echo "start runs with experiment tag: ${EXPERIMENT_TAG}"
			experiment_type=${BO_exps[${acq_string}]}
			echo "and experiment type: ${experiment_type}"
			#bash launch_foraging_BO.sh "${datafolder}" "${experiment_type}" "${EXPERIMENT_TAG}" "" "Large" ""
			bash launch_foraging_BO.sh "${datafolder}" "${experiment_type}" "${EXPERIMENT_TAG}" "" "Large" "2"
	done
    done
done

