#!/bin/bash -e



#two args: data-dir and number of trials


data=$1
export Generation=30000

echo "doing generation ${Generation}"
sleep 2.5

# Create a data diretory
mkdir -p $data
declare -A descriptors
declare -A voronoi
declare -A behav

behav["Aggregation"]="SWARM_AGGREGATION"
behav["Dispersion"]="SWARM_DISPERSION"
behav["DecayCoverage"]="SWARM_COVERAGE"
behav["DecayBorderCoverage"]="SWARM_BORDERCOVERAGE"
behav["Flocking"]="SWARM_FLOCKING"

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

command="bin/ite_swarms_"
bo_executable="bin/BO"
# note: cvt and 10D does not really matter since we are not evolving

#descriptors["Gomes_sdbc_walls_and_robots_std"]=10
#voronoi["Gomes_sdbc_walls_and_robots_std"]="cvt"

#descriptors["environment_diversity"]=6
#voronoi["environment_diversity"]=""


descriptors["history"]=3
voronoi["history"]=""

#descriptors["cvt_rab_spirit"]=1024
#voronoi["cvt_rab_spirit"]="cvt"

# descriptors["baseline"]=""
# voronoi["baseline"]=""


time["DecayCoverage"]=400
time["DecayBorderCoverage"]=400
time["Dispersion"]=400
time["Aggregation"]=400
time["Flocking"]=400

perturbations_folder="experiments/perturbations"
# for FaultType in "FAULT_PROXIMITYSENSORS_SETMIN" "FAULT_PROXIMITYSENSORS_SETMAX" "FAULT_PROXIMITYSENSORS_SETRANDOM" \
# "FAULT_ACTUATOR_LWHEEL_SETHALF" "FAULT_ACTUATOR_RWHEEL_SETHALF" "FAULT_ACTUATOR_BWHEELS_SETHALF"; do
for FaultIndex in $(seq 0 2); do
	SimTime=${time[${FitfunType}]}
	echo "simtime"${SimTime}
	for FaultID in "-1"; do
		for FitfunType in Flocking; do
			echo 'Fitfun'${FitFunType}
			for SensorRange in 0.11; do
				echo 'sens'${SensorRange}
				for key in ${!descriptors[@]}; do
					DescriptorType=${key}
					BD_DIMS=${descriptors[${key}]}
					CVT=${voronoi[${DescriptorType}]}

					tag=${CVT}${BD_DIMS}D
					SwarmBehaviour="/"
					sleep 5


					echo "doing ${DescriptorType} now"
					echo "has ${BD_DIMS} dimensions"
					echo "tag is ${tag}"

					for Replicates in $(seq 1 5); do

						# Take template.argos and make an .argos file for this experiment
						SUFFIX=${Replicates}

						#read the data from the original experiment
						Base=${data}/${FitfunType}range${SensorRange}/${DescriptorType}
						# look at archive dir at previous perturbation results; config is at FAULT_NONE
                        FaultType="FILE:${perturbations_folder}/run${Replicates}_p${FaultIndex}.txt"
                        ConfigFolder=${Base}/faultyrun${Replicates}_p${FaultIndex}

						mkdir -p ${ConfigFolder}
						ConfigFile=${ConfigFolder}/ITEexp_${SUFFIX}.argos
						ArchiveDir=${Base}/results${SUFFIX} # point to the generation file and archive
						archivefile="${ArchiveDir}/archive_${FINALGEN_ARCHIVE}.dat"
						Outfolder=${ConfigFolder}/BOresults${SUFFIX}

                        
						mkdir -p $Outfolder
						echo "config ${ConfigFile}"
						touch ${ConfigFile}
						sed -e "s|THREADS|0|" \
							-e "s|TRIALS|${2}|" \
							-e "s|ROBOTS|10|" \
							-e "s|EXPERIMENT_LENGTH|${SimTime}|" \
							-e "s|SEED|${Replicates}|" \
							-e "s|FITFUN_TYPE|${FitfunType}|" \
							-e "s|DESCRIPTOR_TYPE|${DescriptorType}|" \
							-e "s|OUTPUTFOLDER|${Outfolder}|" \
							-e "s|SENSOR_RANGE|${SensorRange}|" \
							-e "s|CENTROIDSFOLDER|experiments/centroids|" \
							-e "s|NOISE_LEVEL|0.05|" \
							-e "s|BEHAVIOUR_TAG|BO${tag}|" \
							-e "s|FAULT_TYPE|${FaultType}|" \
							-e "s|FAULT_ID|${FaultID}|" \
							-e "s|SWARM_BEHAV|${SwarmBehaviour}|" \
							experiments/experiment_template_perturbation.argos \
							>${ConfigFile}

						if [ ! -z "${CVT}" ]; then
							echo ${CVT}
						fi

						# Call ARGoS
						export COMMAND=${command}${tag}
                        export BO_OutputFolder=${Outfolder}/BO_output
                        export ArchiveFolder=${ArchiveDir}
                        export BO_Executable=${bo_executable}${tag}
                        export ConfigFile

						echo "submitting job"
						#bash zero_padding_data.sh ${Base}/results${SUFFIX} # make sure everything is zero-padded
                        bash submit_ite.sh
					done
				done
			done
		done
	done
done
