#!/bin/bash

datafolder=$1
if [ -z "${datafolder}" ]; then
	echo "Error: no datafolder given"
	exit 125
fi
declare -A BO_exps
#acq_funs[0]="UCB"
acq_funs[0]="UCB_LOCAL3"
#acq_funs[3]="UCB"
#acq_funs[1]="UCB_ID"

#BO_exps["UCB"]="random_single_record"
BO_exps["UCB_LOCAL3"]="BO_single_multi"
#BO_exps["UCB_ID"]="BO_single_IDprior"
for alpha in 0.93; do
    for l in  0.12; do
        for acq in ${!acq_funs[@]};do
		for delay_prob in 0 20 40 60 80; do
			for waituntil in "false" "true"; do
		        #bash cmake_scripts/make_all_heterosim.sh $acq 0 $alpha $l
			acq_string=${acq_funs[${acq}]}
			export EXPERIMENT_TAG="alpha${alpha}_l${l}_${acq_string}_M52VarNoise"
			echo "start runs with experiment tag: ${EXPERIMENT_TAG}"
			experiment_type=${BO_exps[${acq_string}]}
			echo "and experiment type: ${experiment_type}"
			#bash launch_foraging_BO.sh "${datafolder}" "${experiment_type}" "${EXPERIMENT_TAG}" "" "Large" "" "0" "0"
			bash launch_foraging_BO.sh "${datafolder}" "${experiment_type}" "${EXPERIMENT_TAG}" "" "Large" "" "${delay_prob}" "${waituntil}"
			done
		done
	done
    done
done

