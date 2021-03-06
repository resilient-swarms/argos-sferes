#!/bin/bash -e

#RESUME_GENERATION=400
set_latest () {
  eval "latest=\${$#}"
}
calc() { awk "BEGIN{print $*}"; }



ceiling_divide() {  
  awk "BEGIN{print int( ($1/$2) + 1 )}";
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

descriptors["environment_diversity"]=7
voronoi["environment_diversity"]=""
time["DecayCoverage"]=400
time["DecayBorderCoverage"]=400
time["Dispersion"]=400
time["Aggregation"]=400
time["Flocking"]=400

export TASK_TYPE="task_agnost"
FitfunType="TaskAgnost"
    echo 'Fitfun'${FitfunType}
    SimTime=${time[${FitfunType}]}
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
			

			#	cp ${ConfigFolder}/results1/centroids_1000_${BD_DIMS}.dat ${Outfolder}
			if [ ! -z "${CVT}"  ]; then
				echo ${CVT}
				vsuffix='-v '${CVT}
				echo ${cvsuffix}
			fi 
		# Take template.argos and make an .argos file for this experiment
		SUFFIX=${Replicates}
		ConfigFolder=${data}/${FitfunType}range${SensorRange}/${DescriptorType}
		Outfolder=${ConfigFolder}/results${SUFFIX}
		ConfigFile=${ConfigFolder}/exp_${SUFFIX}  # no .argos tag here, will add filenumber and .argos later         

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
		if [[ "${RESUME_GENERATION}" = "${Outfolder}/gen_*" ]]; then
				echo "no previous generation file found"
				DO_CONFIG=true
		else
				echo "found last generation file: "${RESUME_GENERATION}
				export GENERATION_FILE=${RESUME_GENERATION}
		fi
		
		if [[ "${DO_CONFIG}"="true" ]] ; then   
				count1=0
				for MaxSpeed in 5 10 15 20 ; do
					count1=$((count1+1))
					count2=0
				
					for Robots in 5 10 15 20; do
						count2=$((count2+1))
						count3=0

						for Wall in 3 4 5 6; do
							count3=$((count3+1))
							count4=0
							# from the wall size, calc the center, arena, halfwall, and the wall_off
							WallThickness=1.0
							HalfWall=$(calc ${Wall}/2)  #
							Arena=$(calc ${Wall}+2*${WallThickness}) # wall + 2m to account for 2*1m wall
							Center=${HalfWall}
							WallOff=$(calc ${Wall}-0.5) # just for init robots

							

							FullWall=$(calc ${Wall}+${WallThickness}/2.0)

							for Cylinder in 0 2 4 6; do
								count4=$((count4+1))
								count5=0

								for RabRange in 0.25 0.50 1.0 2.0; do
									count5=$((count5+1))
									count6=0
									TwoR=$(calc 2*${RabRange})
									RabGrid=$(ceiling_divide ${Wall} ${TwoR})  #  divide arena spanned by walls into cells of 2R
									# take ceiling in case not divisible (e.g., wall=5 and 2R=4, take 2 grid cells per dimension)
									for ProxiRange in 0.055 0.11 0.22 0.44; do   # 4800 combinations in total
                                        for RealFitfunType in Aggregation Dispersion DecayCoverage DecayBorderCoverage Flocking ; do   
                                            count7=$((count7+1))
                                            ConfigTag="${count1},${count2},${count3},${count4},${count5},${count6},${count7}"
                                            #echo ${ConfigTag}
                                            mkdir -p $Outfolder
                                            sed -e "s|THREADS|0|" \
                                                    -e "s|TRIALS|50|" \
                                                    -e "s|ROBOTS|${Robots}|"                    \
                                                    -e "s|EXPERIMENT_LENGTH|${SimTime}|" \
                                                    -e "s|SEED|${Replicates}|"                    \
                                                    -e "s|FITFUN_TYPE|${RealFitfunType}|"                   \
                                                    -e "s|DESCRIPTOR_TYPE|${DescriptorType}|"                  \
                                                    -e "s|OUTPUTFOLDER|${Outfolder}|" \
                                                    -e "s|SENSOR_RANGE|${ProxiRange}|" \
                                                    -e "s|CENTROIDSFOLDER|experiments/centroids| " \
                                                    -e "s|NOISE_LEVEL|0.05|"    \
                                                    -e "s|BEHAVIOUR_TAG|${tag}|" \
                                                    -e "s|MAX_SPEED|${MaxSpeed}|"\
                                                    -e "s|ARENA,ARENA|${Arena},${Arena}|"\
                                                    -e "s|CENTER,CENTER|${Center},${Center}|" \
                                                    -e "s|FULL_WALL|${Wall}|"\
                                                    -e "s|HALF_WALL|${HalfWall}|"\
                                                    -e "s|WALL_OFF,WALL_OFF|${WallOff},${WallOff}|"\
                                                    -e "s|NUM_CYLINDERS|${Cylinder}|"\
                                                    -e "s|RAB_RANGE|${RabRange}|"\
                                                    -e "s|RAB_GRID,RAB_GRID|${RabGrid},${RabGrid}|"\
                                            experiments/environment_template.argos \
                                                    > ${ConfigFile}_${ConfigTag}.argos
                                        done
									done
								done
							done
						done
					done
				done
			fi

		echo "submitting job"
		sbatch submit_envir_job.sh 
		done
	done

