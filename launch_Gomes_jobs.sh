#!/bin/bash -e

#RESUME_GENERATION=400
set_latest() {
	eval "latest=\${$#}"
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

#descriptors["cvt_spirit"]=400
#voronoi["cvt_spirit"]="cvt"
#descriptors["multiagent_spirit"]=576
#voronoi["multiagent_spirit"]="cvt"

descriptors["Gomes_sdbc_walls_and_robots_std"]=10
voronoi["Gomes_sdbc_walls_and_robots_std"]="cvt"
time["DecayCoverage"]=400
time["DecayBorderCoverage"]=400
time["Dispersion"]=400
time["Aggregation"]=400
time["Flocking"]=400

for FitfunType in Aggregation Dispersion DecayCoverage DecayBorderCoverage Flocking; do # add Flocking later
	echo 'Fitfun'${FitfunType}
	SimTime=${time[${FitfunType}]}
	echo "simtime"${SimTime}
	for SensorRange in 0.11; do
		echo 'sens'${SensorRange}
		for key in ${!descriptors[@]}; do
			DescriptorType=${key}
			BD_DIMS=${descriptors[${key}]}
			CVT=${voronoi[${DescriptorType}]}
			tag=${CVT}${BD_DIMS}D
			echo "doing ${DescriptorType} now"
			echo "has ${BD_DIMS} dimensions"
			echo "tag is ${tag}"

			for Replicates in $(seq 1 5); do

				# Take template.argos and make an .argos file for this experiment
				SUFFIX=${Replicates}
				ConfigFolder=${data}/${FitfunType}range${SensorRange}/${DescriptorType}
				Outfolder=${ConfigFolder}/results${SUFFIX}
				ConfigFile=${ConfigFolder}/exp_${SUFFIX}.argos

				mkdir -p $Outfolder
				sed -e "s|THREADS|0|" \
					-e "s|TRIALS|50|" \
					-e "s|ROBOTS|10|" \
					-e "s|EXPERIMENT_LENGTH|${SimTime}|" \
					-e "s|SEED|${Replicates}|" \
					-e "s|FITFUN_TYPE|${FitfunType}|" \
					-e "s|DESCRIPTOR_TYPE|${DescriptorType}|" \
					-e "s|OUTPUTFOLDER|${Outfolder}|" \
					-e "s|SENSOR_RANGE|${SensorRange}|" \
					-e "s|CENTROIDSFOLDER|experiments/centroids| " \
					-e "s|NOISE_LEVEL|0.05|" \
					-e "s|BEHAVIOUR_TAG|${tag}|" \
					experiments/Gomes_experiment_template.argos \
					>${ConfigFile}
				if [ ! -z "${CVT}" ]; then
					echo ${CVT}
					#if [ $Replicates -eq 1 ]; then
					#	python sferes2/modules/cvt_map_elites/cvt.py -k 1000 -d ${BD_DIMS} -p 100000 -f ${Outfolder}
					#else

					#	cp ${ConfigFolder}/results1/centroids_1000_${BD_DIMS}.dat ${Outfolder}
					#fi
					vsuffix='-v '${CVT}
					echo ${cvsuffix}

				fi

				# Call ARGoS
				export tag
				export BD=${BD_DIMS}
				export CONFIG=${ConfigFile}
				export OUTPUTDIR=${Outfolder}
				export VORONOI=${CVT}
				bash zero_padding_data.sh ${Outfolder}
				set_latest ${Outfolder}/gen_*

				#Comment below line if you start experiments (no previous generation files)
				RESUME_GENERATION=${latest}
				if [[ "${RESUME_GENERATION}" == "${Outfolder}/gen_*" ]]; then
					echo "no previous generation file found"
					export GENERATION_FILE=""
				else
					echo "found last generation file: "${RESUME_GENERATION}
					export GENERATION_FILE=${RESUME_GENERATION}
				fi
				if [[ $GENERATION_FILE == *10100 ]]; then
					echo "skipping this one, already finished"
				else
					sbatch submit_job.sh
				fi
			done
		done
	done
done
