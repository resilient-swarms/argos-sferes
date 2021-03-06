#!/bin/bash -e

data=$1
video=$3
if [ "$video" = "video" ]; then	
	if [ "$2" = "best" ] || [ "$2" = "impact" ]; then   # testing faulty best or normal best on faulty scenario  -> give numbers to track faults
		template_file="experiments/experiment_template_perturbation_with_visual_numbered.argos"
	else
		template_file="experiments/experiment_template_perturbation_with_visual.argos"   # testing on normal environment; no need to track
	fi
	
else
	template_file="experiments/experiment_template_perturbation.argos"
fi
echo "will use template file ${template_file}"




export FINALGEN_ARCHIVE=30000 # never forget zero-padding for generation file, not for archive file
export FINALGEN_GENFILE=30000

echo "doing generation ${FINALGEN_ARCHIVE}"
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

command="bin/analysis" # note: cvt and 10D does not really matter since we are not evolving

descriptors["Gomes_sdbc_walls_and_robots_std"]=10
voronoi["Gomes_sdbc_walls_and_robots_std"]="cvt"

descriptors["environment_diversity"]=6
voronoi["environment_diversity"]=""


descriptors["history"]=3
voronoi["history"]=""

descriptors["cvt_rab_spirit"]=1024
voronoi["cvt_rab_spirit"]="cvt"

#descriptors["baseline"]=""
#voronoi["baseline"]=""


time["DecayCoverage"]=400
time["DecayBorderCoverage"]=400
time["Dispersion"]=400
time["Aggregation"]=400
time["Flocking"]=400

perturbations_folder="experiments/perturbations"
# for FaultType in "FAULT_PROXIMITYSENSORS_SETMIN" "FAULT_PROXIMITYSENSORS_SETMAX" "FAULT_PROXIMITYSENSORS_SETRANDOM" \
# "FAULT_ACTUATOR_LWHEEL_SETHALF" "FAULT_ACTUATOR_RWHEEL_SETHALF" "FAULT_ACTUATOR_BWHEELS_SETHALF"; do
for FaultIndex in $(seq 12 12); do
	SimTime=${time[${FitfunType}]}
	echo "simtime"${SimTime}
	for FaultID in "-1"; do
		for FitfunType in Aggregation; do
			echo 'Fitfun'${FitFunType}
			for SensorRange in 0.11; do
				echo 'sens'${SensorRange}
				for key in ${!descriptors[@]}; do
					DescriptorType=${key}
					BD_DIMS=${descriptors[${key}]}
					CVT=${voronoi[${DescriptorType}]}
					if [ "$DescriptorType" = "baseline" ]; then
						tag=""
						SwarmBehaviour=${behav[${FitfunType}]}
						echo "SwarmBehaviour = "${SwarmBehaviour}
						sleep 5
					else
						tag=${CVT}${BD_DIMS}DANA
						SwarmBehaviour="/"
						sleep 5
					fi
					echo "doing ${DescriptorType} now"
					echo "has ${BD_DIMS} dimensions"
					echo "tag is ${tag}"

					for Replicates in $(seq 4 4); do

						# Take template.argos and make an .argos file for this experiment
						SUFFIX=${Replicates}

						#read the data from the original experiment
						Base=${data}/${FitfunType}range${SensorRange}/${DescriptorType}
						# look at archive dir at FAULT_NONE; config includes perturbations

						
						#write data to these folders
						if [ "$2" = "best" ]; then
							echo "will look for perturbations at run${Replicates}_p${FaultIndex}"
							ConfigFolder=${Base}
							ConfigFile=${ConfigFolder}/history_exp_${Replicates}_p${FaultIndex}.argos # just to write the history
							ArchiveDir=${ConfigFolder}/faultyrun${Replicates}_p${FaultIndex}/
							export Outfolder=${ArchiveDir}/${video}/results${SUFFIX} # where to output the results
							export Searchfolder=${ArchiveDir}/results${SUFFIX}  # where to search for best indiv
							FaultType="FILE:${perturbations_folder}/run${Replicates}_p${FaultIndex}.txt"
                        			elif [ "$2" = "best_compare" ]; then   # compare the best one but evaluate it in the no-fault environment
							echo "will look for perturbations at run${Replicates}_p${FaultIndex}"
							ConfigFolder=${Base}
							ConfigFile=${ConfigFolder}/history_exp_${Replicates}_p${FaultIndex}.argos # just to write the history
							ArchiveDir=${ConfigFolder}/faultyrun${Replicates}_p${FaultIndex}/
							export Outfolder=${ArchiveDir}/${video}/results${SUFFIX} # where to output the results
							export Searchfolder=${ArchiveDir}/results${SUFFIX}  # where to search for best indiv
							FaultType="FAULT_NONE"
                        			elif [ "$2" = "impact" ]; then  # assess impact of fault on the normal individual
							echo "will look for perturbations at run${Replicates}_p${FaultIndex}"
							FaultType="FILE:${perturbations_folder}/run${Replicates}_p${FaultIndex}.txt"
						    	ConfigFolder=${Base}/faultyrun${Replicates}_p${FaultIndex}
							mkdir -p ${ConfigFolder}
							ConfigFile=${ConfigFolder}/exp_${SUFFIX}.argos
							ArchiveDir=${ConfigFolder}/faultyrun${Replicates}_p${FaultIndex}/
							export Outfolder=${ArchiveDir}/${video}/results${SUFFIX} # where to output the results
							export Searchfolder=${Base}/FAULT_NONE/results${SUFFIX}  # where to search for best indiv (here search for the FAULT_NONE)					
						else
							# look at archive dir at previous perturbation results; config is at FAULT_NONE
						    	FaultType="FILE:${perturbations_folder}/run${Replicates}_p${FaultIndex}.txt"
						    	ConfigFolder=${Base}/faultyrun${Replicates}_p${FaultIndex}
							mkdir -p ${ConfigFolder}
							ConfigFile=${ConfigFolder}/exp_${SUFFIX}.argos
							export ArchiveDir=${Base}/results${SUFFIX} # point to the generation file and archive
							export archivefile="${ArchiveDir}/archive_${FINALGEN_ARCHIVE}.dat"
							Outfolder=${ConfigFolder}/results${SUFFIX}
						fi
						mkdir -p $Outfolder
						echo "config ${ConfigFile}"
						touch ${ConfigFile}
						sed -e "s|THREADS|0|" \
							-e "s|TRIALS|${3}|" \
							-e "s|ROBOTS|10|" \
							-e "s|EXPERIMENT_LENGTH|${SimTime}|" \
							-e "s|SEED|${Replicates}|" \
							-e "s|FITFUN_TYPE|${FitfunType}|" \
							-e "s|DESCRIPTOR_TYPE|analysis|" \
							-e "s|OUTPUTFOLDER|${Outfolder}|" \
							-e "s|SENSOR_RANGE|${SensorRange}|" \
							-e "s|CENTROIDSFOLDER|experiments/centroids|" \
							-e "s|NOISE_LEVEL|0.05|" \
							-e "s|BEHAVIOUR_TAG|${tag}|" \
							-e "s|FAULT_TYPE|${FaultType}|" \
							-e "s|FAULT_ID|-1|" \
							-e "s|SWARM_BEHAV|/|" \
							${template_file} \
							>${ConfigFile}
						if [ "$DescriptorType" = "baseline" ]; then
								echo "changing loopfunction"
								sed -i "s|evolution_loop|baseline-behavs-loop|" ${ConfigFile}
								sed -i "s|nn_controller|baseline-behavs|" ${ConfigFile}
								sed -i "s|tnn|bb|" ${ConfigFile}
								if [ "$FitfunType" = "Flocking" ]; then
									# halve speeds and double simtime
									sed -i 's|ticks_per_second="5"|ticks_per_second="10"|' ${ConfigFile}
									sed -i 's|max_speed="10"|max_speed="5"|' ${ConfigFile}
									sed -i 's|iterations="5"|iterations="10"|' ${ConfigFile}
								fi
						fi
						if [ ! -z "${CVT}" ]; then
							echo ${CVT}
						fi

						# Call ARGoS
						export COMMAND=${command}${tag}
						export CONFIG=${ConfigFile}
						export OUTPUTDIR=${Outfolder}
						export generationfile="${Base}/results${SUFFIX}/gen_${FINALGEN_GENFILE}"

						echo "submitting job"
						bash zero_padding_data.sh ${Base}/results${SUFFIX} # make sure everything is zero-padded
						if [ "$DescriptorType" = "baseline" ]; then
							echo "will submit baseline job"
							bash submit_baseline_job.sh   # just record the performance of the baseline controllers in the faulty environment
						else
							if [ "${video}" = "video" ]; then
								if [ "$2" = "best" ]; then
									export VIDEOFILE="/home/david/Videos/${FitfunType}_${DescriptorType}_FAULT${FaultIndex}_run${Replicates}"   #
								elif [ "$2" = "best_compare" ]; then
									export VIDEOFILE="/home/david/Videos/${FitfunType}_${DescriptorType}_FAULT${FaultIndex}_run${Replicates}_NORMAL"   #
								elif [ "$2" = "impact" ]; then
									export VIDEOFILE="/home/david/Videos/${FitfunType}_${DescriptorType}_FAULT${FaultIndex}_run${Replicates}_IMPACT"   #
									
								fi
								bash submit_test.sh "video"
							elif [ "$2" = "best" ]; then
								bash submit_test.sh $2 # submit in your own system; 7Zip support needed+jobs are short
							else
								sbatch submit_test.sh $2 # submit to iridis
							fi
						fi
					done
				done
			done
		done
	done
done
