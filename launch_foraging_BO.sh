#!/bin/bash -e

#one: data-dir

data=$1
run_type=$2
if [ "$run_type" = "virtual" ]; then
    UseVirtual="True"
	TopOutputFolder="virtual_energy_exp"
    command="bin/ite_swarms_"
elif [ "$run_type" = "random" ]; then
    command="bin/ite_baselines_"
    export BaselineChoice=" -b ${run_type}"
    TopOutputFolder="baselines"
elif [ "$run_type" = "gradient" ]; then
    command="bin/ite_baselines_"
    export BaselineChoice=" -b ${run_type}"
    TopOutputFolder="baselines"
else
    command="bin/ite_swarms_"
fi
output_tag=$3

export Generation=20000

echo "doing generation ${Generation}"
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

SimTime=120

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
                tag=BO${CVT}${BD_DIMS}DREAL
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
                    if [ "$run_type" = "BO" ] || [ "$run_type" = "virtual" ]; then
                        export BO_OutputFolder=${Outfolder}/BO_output${output_tag}
                    else 
                        export BO_OutputFolder=${Outfolder}${output_tag}
                    fi
                    rm -rf ${BO_OutputFolder}
                    rm ${Outfolder}/fitness
                    mkdir -p $Outfolder
                    echo "config ${ConfigFile}"
                    touch ${ConfigFile}
                    sed -e "s|THREADS|0|" \
                        -e "s|TRIALS|8|" \
                        -e "s|ROBOTS|${robots}|" \
                        -e "s|EXPERIMENT_LENGTH|${SimTime}|" \
                        -e "s|SEED|${Replicates}|" \
                        -e "s|FITFUN_TYPE|${FitfunType}|" \
                        -e "s|DESCRIPTOR_TYPE|${DescriptorType}|" \
                        -e "s|OUTPUTFOLDER|${Outfolder}|" \
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
                        experiments/harvesting/harvesting_template.argos \
                        >${ConfigFile}

                    if [ ! -z "${CVT}" ]; then
                        echo ${CVT}
                    fi

                    # Call ARGoS
                    export COMMAND=${command}${tag}
                    export ArchiveFolder=${ArchiveDir}
                    export BO_Executable=${bo_executable}${tag}
                    export ConfigFile

                    echo "submitting ite job"
                    bash submit_ite.sh
                done
            done
        done
    done
done
