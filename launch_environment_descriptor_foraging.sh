#!/bin/bash -e

#RESUME_GENERATION=400
set_latest() {
	eval "latest=\${$#}"
}
calc() { awk "BEGIN{print $*}"; }

ceiling_divide() {
	awk "BEGIN{print int( ($1/$2) + 1 )}"
}
source activate py3.7 # just for the cvt initialisation

data=$1
if [ -z "${data}" ]; then
	echo "Error: no datafolder given"
	exit 125
fi
# Create a data diretory
mkdir -p $data
declare -A descriptors
declare -A voronoi

descriptors["environment_diversity"]=6
voronoi["environment_diversity"]=""
time["Foraging"]=120

export TASK_TYPE="task_specific"

for FitfunType in Foraging; do
	echo 'Fitfun'${FitfunType}
	SimTime=${time[${FitfunType}]}
	echo "simtime"${SimTime}
	for key in ${!descriptors[@]}; do
		DescriptorType=${key}
		BD_DIMS=${descriptors[${key}]}
		CVT=${voronoi[${DescriptorType}]}
		export tag=${CVT}${BD_DIMS}DREAL
		echo "doing ${DescriptorType} now"
		echo "has ${BD_DIMS} dimensions"
		echo "tag is ${tag}"

		for Replicates in $(seq 1 5); do

			#	cp ${ConfigFolder}/results1/centroids_1000_${BD_DIMS}.dat ${Outfolder}
			if [ ! -z "${CVT}" ]; then
				echo ${CVT}
				vsuffix='-v '${CVT}
				echo ${cvsuffix}
			fi
			# Take template.argos and make an .argos file for this experiment
			SUFFIX=${Replicates}
			ConfigFolder=${data}/${FitfunType}/${DescriptorType}
			Outfolder=${ConfigFolder}/results${SUFFIX}
			ConfigFile=${ConfigFolder}/exp_${SUFFIX} # no .argos tag here, will add filenumber and .argos later

			export BD=${BD_DIMS}
			export CONFIG=${ConfigFile}
			export OUTPUTDIR=${Outfolder}
			export VORONOI=${CVT}
			export REDIRECT=${data}/run${Replicates}
			mkdir -p ${REDIRECT}
			bash zero_padding_data.sh ${Outfolder}
			set_latest ${Outfolder}/gen_*

			#check if you need to resume experiments
			RESUME_GENERATION=${latest}
			if [[ "${RESUME_GENERATION}" == "${Outfolder}/gen_*" ]]; then
				echo "no previous generation file found"
				export GENERATION_FILE=""
				DO_CONFIG="true"
			else
				echo "found last generation file: "${RESUME_GENERATION}
				export GENERATION_FILE=${RESUME_GENERATION}
				DO_CONFIG="false"
			fi

			if [[ "${DO_CONFIG}" == "true" ]]; then
				echo "doing config"
				count1=0
				for MaxSpeed in 5 10 15 20; do
					count1=$((count1 + 1))
					count2=0

					for Robots in 5 10 15 20; do
						count2=$((count2 + 1))
						count3=0

						for Wall in 3 4 5 6; do
							count3=$((count3 + 1))
							count4=0
							# from the wall size, calc the center, arena, halfwall, and the wall_off
							WallThickness=1.0
							HalfWall=$(calc ${Wall}/2)               #
							Arena=$(calc ${Wall}+2*${WallThickness}) # wall + 2m to account for 2*1m wall
							Center=${HalfWall}
							WallOff=$(calc ${Wall}-0.5) # just for init robots

							FullWall=$(calc ${Wall}+${WallThickness}/2.0)

							for Cylinder in 0 2 4 6; do
								count4=$((count4 + 1))
								count5=0

								for GroundNoise in 10 20 40 80; do
									count5=$((count5 + 1))
									count6=0
									for ProxiRange in 0.055 0.11 0.22 0.44; do # 4800 combinations in total
										count6=$((count6 + 1))
										ConfigTag="${count1},${count2},${count3},${count4},${count5},${count6}"
										#echo ${ConfigTag}
										mkdir -p $Outfolder
										sed -e "s|THREADS|0|" \
											-e "s|TRIALS|8|" \
											-e "s|ROBOTS|${Robots}|" \
											-e "s|GROUND_NOISE|${GroundNoise}|" \
											-e "s|EXPERIMENT_LENGTH|${SimTime}|" \
											-e "s|SEED|${Replicates}|" \
											-e "s|FITFUN_TYPE|${FitfunType}|" \
											-e "s|DESCRIPTOR_TYPE|${DescriptorType}|" \
											-e "s|OUTPUTFOLDER|${Outfolder}|" \
											-e "s|SENSOR_RANGE|${ProxiRange}|" \
											-e "s|CENTROIDSFOLDER|experiments/centroids| " \
											-e "s|NOISE_LEVEL|0.05|" \
											-e "s|BEHAVIOUR_TAG|${tag}|" \
											-e "s|MAX_SPEED|${MaxSpeed}|" \
											-e "s|ARENA,ARENA|${Arena},${Arena}|" \
											-e "s|ARENA,|${Arena},|" \
											-e "s|CENTER,CENTER|${Center},${Center}|" \
											-e "s|FULL_WALL|${FullWall}|" \
											-e "s|HALF_WALL|${HalfWall}|" \
											-e "s|WALL_OFF,WALL_OFF|${WallOff},${WallOff}|" \
											-e "s|NUM_CYLINDERS|${Cylinder}|" \
											-e "s|RAB_RANGE|${RabRange}|" \
											-e "s|RAB_GRID,RAB_GRID|${RabGrid},${RabGrid}|" \
											experiments/harvesting/environment_template_harvesting.argos \
											>${ConfigFile}_${ConfigTag}.argos
									done
								done
							done
						done
					done
				done
			fi

			echo "submitting job"
			if [[ $GENERATION_FILE == *40100 ]]; then
				echo "skipping this one, already finished"
			else
				bash submit_envir_job.sh
			fi

		done
	done
done
