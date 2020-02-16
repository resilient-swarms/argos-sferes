#!/bin/bash -e

#RESUME_GENERATION=400
set_latest() {
	eval "latest=\${$#}"
}
source activate py3.7 # just for the cvt initialisation

data=$1

# Create a data diretory
mkdir -p $data
declare -A descriptors
declare -A voronoi

#descriptors["cvt_spirit"]=400
#voronoi["cvt_spirit"]="cvt"
#descriptors["multiagent_spirit"]=576
#voronoi["multiagent_spirit"]="cvt"

descriptors["history"]=3
voronoi["history"]=""
SimTime=120
FitfunType=Foraging

echo 'Fitfun'${FitfunType}
echo "simtime"${SimTime}
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
		ConfigFolder=${data}/${FitfunType}/${DescriptorType}
		Outfolder=${ConfigFolder}/results${SUFFIX}
		ConfigFile=${ConfigFolder}/exp_${SUFFIX}.argos

		mkdir -p $Outfolder
		sed -e "s|THREADS|0|" \
			-e "s|TRIALS|8|" \
			-e "s|ROBOTS|6|" \
			-e "s|EXPERIMENT_LENGTH|${SimTime}|" \
			-e "s|SEED|${Replicates}|" \
			-e "s|FITFUN_TYPE|${FitfunType}|" \
			-e "s|DESCRIPTOR_TYPE|${DescriptorType}|" \
			-e "s|OUTPUTFOLDER|${Outfolder}|" \
			-e "s|CENTROIDSFOLDER|experiments/centroids|" \
			-e "s|SENSOR_RANGE|0.11|" \
			-e "s|NOISE_LEVEL|0.05|" \
			-e "s|GROUND_NOISE|20|"  \
			-e "s|BEHAVIOUR_TAG|${tag}|" \
			experiments/harvesting/harvesting_template.argos \
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
		if [[ $GENERATION_FILE == *6000 ]]; then
			echo "skipping this one, already finished"
		else
			echo "submitting job"
			bash submit_job.sh
		fi
	done
done
