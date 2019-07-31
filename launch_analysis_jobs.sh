#!/bin/bash -e



 


data=~/DataFinal/datanew
export FINALGEN=1000
# Create a data diretory
mkdir -p $data
declare -A descriptors
declare -A voronoi


# descriptors["history"]=2
# descriptors["cvt_mutualinfoact"]=14
# descriptors["cvt_mutualinfo"]=21
# descriptors["cvt_spirit"]=400
# descriptors["cvt_sdbc_all_std"]=14
# voronoi["cvt_mutualinfo"]="cvt"
# voronoi["history"]=""
# voronoi["cvt_mutualinfoact"]="cvt"
# voronoi["cvt_mutualinfo"]="cvt"
# voronoi["cvt_spirit"]="cvt"
# voronoi["cvt_sdbc_all_std"]="cvt"


command="bin/analysis"   # note: cvt and 10D does not really matter since we are not evolving


descriptors["Gomes_sdbc_walls_and_robots_std"]=10
voronoi["Gomes_sdbc_walls_and_robots_std"]="cvt"

descriptors["environment_diversity"]=6
voronoi["environment_diversity"]=""


time["DecayCoverage"]=400
time["DecayBorderCoverage"]=400
time["Dispersion"]=400
time["Aggregation"]=400
time["Flocking"]=400

for FaultType in "FAULT_NONE" "FAULT_PROXIMITYSENSORS_SETMIN" "FAULT_PROXIMITYSENSORS_SETMAX" "FAULT_PROXIMITYSENSORS_SETRANDOM" \
"FAULT_ACTUATOR_LWHEEL_SETHALF" "FAULT_ACTUATOR_RWHEEL_SETHALF" "FAULT_ACTUATOR_BWHEELS_SETHALF"; do
SimTime=${time[${FitfunType}]}
echo "simtime"${SimTime}
for FaultID in "0"; do
for FitfunType in Aggregation Dispersion DecayCoverage DecayBorderCoverage Flocking; do
    echo 'Fitfun'${FitFunType}
    for SensorRange in 11; do
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

			#read the data from the original experiment
			Base=${data}/${FitfunType}range${SensorRange}/${DescriptorType}
			ArchiveDir=${Base}/results${SUFFIX}  # point to the generation file and archive
            
			#write data to these folders
			ConfigFolder=${Base}/${FaultType}
			ConfigFile=${ConfigFolder}/exp_${SUFFIX}.argos
            Outfolder=${ConfigFolder}/results${SUFFIX}
	    	
			
	    	mkdir -p $Outfolder
           echo "config ${ConfigFile}"
	    	touch ${ConfigFile} 
        	sed -e "s|THREADS|0|" \
				-e "s|TRIALS|3|" \
                -e "s|ROBOTS|10|"                    \
                -e "s|EXPERIMENT_LENGTH|${SimTime}|" \
				-e "s|SEED|${Replicates}|"                    \
                -e "s|FITFUN_TYPE|${FitfunType}|"                   \
                -e "s|DESCRIPTOR_TYPE|analysis|"                  \
                -e "s|OUTPUTFOLDER|${Outfolder}|" \
				-e "s|SENSOR_RANGE|${SensorRange}|" \
				-e "s|CENTROIDSFOLDER|experiments/centroids|" \
				-e "s|NOISE_LEVEL|0.05|"    \
                -e "s|BEHAVIOUR_TAG|${tag}|" \
				-e "s|FAULT_TYPE|${FaultType}|" \
                -e "s|FAULT_ID|${FaultID}|" \
				experiments/experiment_template_perturbation.argos \
                > ${ConfigFile}
             if [ ! -z "${CVT}"  ]; then
        	echo ${CVT}
			#python sferes2/modules/cvt_map_elites/cvt.py -k 1000 -d ${BD_DIMS} -p 100000 -f ${Outfolder}
			#vsuffix='-v '${CVT}
			#cho ${cvsuffix}
			fi
		
			# Call ARGoS
			export COMMAND=${command}${tag}
			export CONFIG=${ConfigFile}
			export OUTPUTDIR=${Outfolder}
			export ARCHIVEDIR=${ArchiveDir}

			echo "submitting job"
			bash submit_test.sh 
        done
	done
    done
done
done
done

