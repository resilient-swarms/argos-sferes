#!/bin/bash -e

#RESUME_GENERATION=400
set_latest () {
  eval "latest=\${$#}"
}
source activate py3.7 # just for the cvt initialisation 

data=$1

# Create a data diretory
mkdir -p $data


declare -A time
declare -A behaviour


time["DecayCoverage"]=400
time["DecayBorderCoverage"]=400
time["Dispersion"]=400
time["Aggregation"]=400
time["Flocking"]=400

behaviour["Aggregation"]=SWARM_AGGREGATION
behaviour["Dispersion"]=SWARM_DISPERSION
behaviour["DecayCoverage"]=SWARM_COVERAGE
behaviour["DecayBorderCoverage"]=SWARM_BORDERCOVERAGE
behaviour["Flocking"]=SWARM_FLOCKING


DescriptorType=baseline
for FitfunType in BorderCoverage DecayBorderCoverage Flocking ; do  # add Flocking later
    echo 'Fitfun'${FitfunType}
    SimTime=${time[${FitfunType}]}
    BEHAVIOUR=${behaviour[${FitfunType}]}
    echo ${BEHAVIOUR}
    sleep 2
    echo "simtime"${SimTime}
    for SensorRange in 0.11; do
	echo 'sens'${SensorRange}

	for Replicates in $(seq 1 5); do
                       
            # Take template.argos and make an .argos file for this experiment
            SUFFIX=${Replicates}
            ConfigFolder=${data}/${FitfunType}range${SensorRange}/${DescriptorType}
            Outfolder=${ConfigFolder}/results${SUFFIX}
	    ConfigFile=${ConfigFolder}/exp_${SUFFIX}.argos
	
	    mkdir -p $Outfolder
           sed 	-e "s|THREADS|0|" \
				-e "s|TRIALS|50|" \
                -e "s|ROBOTS|10|"                    \
                -e "s|EXPERIMENT_LENGTH|${SimTime}|" \
				-e "s|SEED|${Replicates}|"                    \
                -e "s|FITFUN_TYPE|${FitfunType}|"                   \
                -e "s|DESCRIPTOR_TYPE|${DescriptorType}|"                  \
                -e "s|OUTPUTFOLDER|${Outfolder}|" \
				-e "s|SENSOR_RANGE|${SensorRange}|" \
				-e "s|CENTROIDSFOLDER|experiments/centroids| " \
				-e "s|NOISE_LEVEL|0.05|"    \
                -e "s|evolution_loopfunctionsBEHAVIOUR_TAG.so|baseline-behavs-loopfunc.so|" \
		-e "s|evolution_loopfunctionsBEHAVIOUR_TAG|baseline-behavs-loop-functions|" \
		-e "s|nn_controller|baseline-behavs|" \
		-e 's|"tnn"|"bb"|' \
		-e "s|SWARM_BEHAVIOR|${BEHAVIOUR}|" \
				experiments/Gomes_experiment_template.argos \
                > ${ConfigFile}
             if [ ! -z "${CVT}"  ]; then
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
	   export CONFIG=${ConfigFile}
	   export OUTPUTDIR=${Outfolder}

           bash submit_baseline_job.sh
        done
    done
done
