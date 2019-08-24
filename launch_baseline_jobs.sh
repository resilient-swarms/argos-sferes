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
for FitfunType in Flocking ; do  
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
