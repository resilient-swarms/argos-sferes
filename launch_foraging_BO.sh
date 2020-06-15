#!/bin/bash -e

#one: data-dir

data=$1
run_type=$2
if [ "$run_type" = "virtual" ]; then
    UseVirtual="True"
	TopOutputFolder="virtual_energy_exp"
    command="bin/ite_swarms_"
    SimTime=120
    trials=8
elif [ "$run_type" = "virtual_single" ]; then
    UseVirtual="True"
	TopOutputFolder="virtual_energy_exp"
    command="bin/behaviour_evol"
    SimTime=960
    trials=1
elif [ "$run_type" = "BO_single" ]; then
    UseVirtual="False"
	TopOutputFolder="single_exp"
    command="bin/behaviour_evol"
    SimTime=28800  # 960*max_evals=28,800 with 30 evals
    trials=1
    ticks_per_subtrial=600 #120*5
	num_subtrials=8
	network_binary=bin/BO3DREAL
    network_config=harvesting_printnetwork.argos
	stop=$4
    optimisation="BO"
    reset=true
elif [ "$run_type" = "BO_single_known" ]; then
    UseVirtual="False"
	TopOutputFolder="single_exp"
    command="bin/behaviour_evol"
    SimTime=28800  # 960*max_evals=28,800 with 30 evals
    trials=1
    ticks_per_subtrial=600 #120*5
	num_subtrials=8
	network_binary=bin/BO3DREAL
    network_config=harvesting_printnetwork.argos
	stop=$4
    optimisation="BO"
    reset=true
elif [ "$run_type" = "random_single" ]; then
    UseVirtual="False"
	TopOutputFolder="single_exp_random"
    command="bin/behaviour_evol"
    SimTime=28800   # 960*max_evals=28,800 with 30 evals
    trials=1
    ticks_per_subtrial=600 #120*5
	num_subtrials=8
	network_binary=bin/BO3DREAL
    network_config=harvesting_printnetwork.argos
	stop=""
    optimisation="random"
    reset=true
elif [ "$run_type" = "random_single_record" ]; then
    UseVirtual="False"
	TopOutputFolder="single_exp_random"
    command="bin/behaviour_evol"
    SimTime=120  # 960*max_evals=28,800 with 30 evals
    trials=8
	network_binary=bin/BO3DREAL
    network_config=harvesting_printnetwork.argos
	stop="" # will default to false stopping criterion always
elif [ "$run_type" = "BO_single_record" ]; then
    UseVirtual="False"
	TopOutputFolder="single_exp"
    command="bin/behaviour_evol"
    SimTime=120  # 960*max_evals=28,800 with 30 evals
    trials=8
	network_binary=bin/BO3DREAL
    network_config=harvesting_printnetwork.argos
	stop="" # will default to false stopping criterion always
elif [ "$run_type" = "uniform" ]; then
	TopOutputFolder="uniform"
    command="bin/ite_swarms_uniform_"
    SimTime=120
    trials=8
elif [ "$run_type" = "random" ]; then
    command="bin/ite_baselines_"
    export BaselineChoice=" -b ${run_type}"
    TopOutputFolder="baselines/random"
    SimTime=120
    trials=8
elif [ "$run_type" = "gradient_closest" ]; then
    command="bin/ite_baselines_"
    export BaselineChoice=" -b ${run_type}"
    TopOutputFolder="baselines/gradient_closest"
    SimTime=120
    trials=8
else
    command="bin/ite_swarms_"
fi
output_tag=$3

export Generation=20000

echo "doing generation ${Generation}"
echo "using executable prefix ${command}"
sleep 2.5

# Create a data diretory
mkdir -p $data
declare -A descriptors
declare -A voronoi
declare -A behav

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


bo_executable="bin/"
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



echo "doing generation ${FINALGEN_ARCHIVE}"
sleep 2.5

FitfunType="Foraging"
echo "simtime "${SimTime}
echo "FitfunType "${FitfunType}
perturbations_folder="experiments/harvesting/perturbations"
# for FaultType in "FAULT_PROXIMITYSENSORS_SETMIN" "FAULT_PROXIMITYSENSORS_SETMAX" "FAULT_PROXIMITYSENSORS_SETRANDOM" \
# "FAULT_ACTUATOR_LWHEEL_SETHALF" "FAULT_ACTUATOR_RWHEEL_SETHALF" "FAULT_ACTUATOR_BWHEELS_SETHALF"; do

declare -A faultnum

faultnum["sensor"]=$(seq 1 30)
faultnum["proximity_sensor"]=$(seq 1 20)
faultnum["ground_sensor"]=$(seq 1 20)
faultnum["actuator"]=$(seq 1 20)
faultnum["software"]=$(seq 1 6)   # number of agents  (1,0,0,0,0,0),(0,1,0,0,0,0), ...
faultnum["software_food"]=$(seq 1 6) # number of agents  (1,0,0,0,0,0),(0,1,0,0,0,0), ...
faultnum["food_scarcity"]=1 # (will loop over food as a dummy)
faultnum["agents"]="3 12 24"      # {1,2,...,12} agents included

for FaultCategory in proximity_sensor ground_sensor actuator software software_food food_scarcity agents; do
    faults=${faultnum[${FaultCategory}]}
    for FaultIndex in ${faults}; do
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
                if [ "$run_type" = "BO_single" ] || [ "$run_type" = "random_single" ]; then
                    tag=${CVT}${BD_DIMS}DREAL
                    bd="identification"
                elif [ "$run_type" = "BO_single_known" ]; then
                    tag=${CVT}${BD_DIMS}DREAL
                    bd="perfect_identification"
                elif [ "$run_type" = "BO_single_record" ] || [ "$run_type" = "random_single_record" ]; then
                    tag=${CVT}${BD_DIMS}DREAL_RECORD
                    bd="identification"
                else
                    tag=BO${CVT}${BD_DIMS}DREAL
                    bd=${DescriptorType}
                fi
                SwarmBehaviour="/"
                sleep 5
            fi
            echo "doing ${DescriptorType} now"
            echo "has ${BD_DIMS} dimensions"
            echo "tag is ${tag}"

            for Replicates in $(seq 1 5); do

                # Take template.argos and make an .argos file for this experiment
                SUFFIX=${Replicates}

                #read the data from the original experiment
                Base=${data}/${FitfunType}/${DescriptorType}
                # look at archive dir at FAULT_NONE; config includes perturbations

                # look at archive dir at previous perturbation results; config is at FAULT_NONE
                FaultType="FILE:${perturbations_folder}/run${Replicates}_${FaultCategory}p${FaultIndex}.txt"
                food_loop="-1"
                if [ "$FaultCategory" = "agents" ]; then
                    robots=$FaultIndex
                    fault=FAULT_NONE
                    FaultID=-1
                    echo "agents category"
                elif [ "$FaultCategory" = "software" ]; then
                    robots=6
                    fault=FAULT_SOFTWARE
                    FaultID=$(($FaultIndex - 1))
                    echo "software category"
                elif [ "$FaultCategory" = "software_food" ]; then
                    robots=6
                    fault=FAULT_SOFTWARE_FOOD
                    food_loop="0"
                    FaultID=$(($FaultIndex - 1))
                    echo "software food"
                elif [ "$FaultCategory" = "food_scarcity" ]; then
                    robots=6
                    fault=FAULT_FOOD_SCARCITY
                    food_loop="0 1 2 3 4 5"
                    FaultID=$(($FaultIndex - 1))
                    echo "food scarcity"
                else
                    robots=6
                    fault=$FaultType
                    FaultID=-1
                    echo "other category (e.g., sensor or actuator)"
                fi


                echo "fault ${fault}   robots ${robots}  FaultID  $FaultID "
                for food in ${food_loop}; do
                    echo "doing food ${food}"
                    if [ "${food}" != "-1" ]; then
                        food_nr_filename=$((${food} + 1))
                        food_tag="f${food_nr_filename}"
                        echo "food tag ${food_tag}"
                    else
                        food_tag=""
                        echo "food tag empty"
                    fi
                    ConfigFolder=${Base}/faultyrun${Replicates}_${FaultCategory}p${FaultIndex}${food_tag}
                    mkdir -p ${ConfigFolder}
                    ConfigFile=${ConfigFolder}/exp_${SUFFIX}.argos
                    export ArchiveDir=${Base}/results${SUFFIX} # point to the generation file and archive
                    export archivefile="${ArchiveDir}/archive_${FINALGEN_ARCHIVE}.dat"
                    Outfolder=${ConfigFolder}/results${SUFFIX}/${TopOutputFolder}
                    echo "Outfolder ${Outfolder}"
                    if [ "$run_type" = "BO" ] || [ "$run_type" = "virtual" ] || [ "$run_type" = "uniform" ] \
                    || [ "$run_type" = "BO_single" ] || [ "$run_type" = "virtual_single" ] || [ "$run_type" = "BO_single_record" ] ||
                    [ "$run_type" = "random_single" ] || [ "$run_type" = "random_single_record" ]; then
                        export BO_OutputFolder=${Outfolder}/BO_output${output_tag}
                    else 
                        export BO_OutputFolder=${Outfolder}${output_tag}
                    fi
		            echo "BO outputfolder = ${BO_OutputFolder}"
                    rm ${BO_OutputFolder}/fitness
                    #rm ${Outfolder}/fitness
                    mkdir -p $Outfolder
                    echo "config ${ConfigFile}"
                    touch ${ConfigFile}
                    sed -e "s|THREADS|0|" \
                        -e "s|TRIALS|${trials}|" \
                        -e "s|ROBOTS|${robots}|" \
                        -e "s|EXPERIMENT_LENGTH|${SimTime}|" \
                        -e "s|SEED|${Replicates}|" \
                        -e "s|FITFUN_TYPE|${FitfunType}|" \
                        -e "s|DESCRIPTOR_TYPE|${bd}|" \
                        -e "s|OUTPUTFOLDER|${BO_OutputFolder}|" \
                        -e "s|CENTROIDSFOLDER|experiments/centroids|" \
                        -e "s|SENSOR_RANGE|0.11|" \
                        -e "s|NOISE_LEVEL|0.05|" \
                        -e "s|GROUND_NOISE|20|" \
                        -e "s|BEHAVIOUR_TAG|${tag}|" \
                        -e "s|FAULT_TYPE|${fault}|" \
                        -e "s|FAULT_ID|${FaultID}|" \
                        -e "s|FOOD_ID|${food}|" \
                        -e "s|SWARM_BEHAV|${SwarmBehaviour}|" \
                        -e "s|USE_VIRTUAL|${UseVirtual}|" \
                        -e "s|TICKS_PER_SUB|${ticks_per_subtrial}|" \
		                -e "s|NUM_SUB|${num_subtrials}|" \
		                -e "s|NETWORK_BINARY|${network_binary}|" \
                        -e "s|NETWORK_CONFIG|${Outfolder}/${network_config}|" \
		                -e "s|STOP|${stop}|" \
                        -e "s|OPTIMISATION|${optimisation}|"  \
                        -e "s|RESET|${reset}|" \
                        experiments/harvesting/harvesting_template.argos \
                        >${ConfigFile}
                        if [ ! -z "${network_config}" ]; then

                            touch ${Outfolder}/${network_config}
			                sed -e "s|OUTPUTFOLDER|${BO_OutputFolder}|" \
                                -e "s|CENTROIDSFOLDER|experiments/centroids|" \
                                experiments/harvesting/harvesting_printnetwork_template.argos \
                                > ${Outfolder}/${network_config}
                        fi 
                    if [ ! -z "${CVT}" ]; then
                        echo ${CVT}
                    fi

                    # Call ARGoS
                    export COMMAND=${command}${tag}
                    export ArchiveFolder=${ArchiveDir}
                    export BO_Executable=${bo_executable}${tag}
                    export ConfigFile

                   
                    if [[ "$run_type" == BO_single* ]] || [[ "$run_type" == random_single* ]];then
                         echo "submitting single job"
                         #sleep 1sh
                         sbatch submit_single.sh
                    else
                        echo "submitting ite job"
                        #sleep 10
                        bash submit_ite.sh
                    fi
		
                done
            done
        done
    done
done
