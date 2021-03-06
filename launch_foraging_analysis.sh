#!/bin/bash -e

data=$1
if [ -z "${data}" ]; then
    echo "Error: no datafolder given"
    exit 125
fi
large=$3
scale=$4
video=$5
UseVirtual=$6

SimTime=120

if [ "$UseVirtual" = "True" ]; then
    VirtualFolder="virtual_energy_exp/exhaustive"
fi


    if [[ $large == "Large" ]]; then
        echo "will do large arena"

        if [ ! -z $scale ]; then
            TemplateFile="experiments/harvesting/harvesting_template_large${scale}X.argos"
            echo "using template: ${TemplateFile}"
            output_tag=${output_tag}_${scale}X
            SimTime=$(($scale * ${SimTime}))
            fault_tag="${scale}X"
        else
            TemplateFile="experiments/harvesting/harvesting_template_large.argos"
            echo "using template: ${TemplateFile}"
            scale="1"
        fi
    else
        echo "will do small arena"
        TemplateFile="experiments/harvesting/harvesting_template.argos"
        echo "using template: ${TemplateFile}"
        scale="1"

    fi

echo "will use template file ${TemplateFile}"

export FINALGEN_ARCHIVE=20000 # never forget zero-padding for generation file, not for archive file
export FINALGEN_GENFILE=20000

echo "doing generation ${FINALGEN_ARCHIVE}"
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

command="bin/analysis" # note: cvt and 10D does not really matter since we are not evolving

#descriptors["cvt_ground_spirit"]=768
#voronoi["cvt_ground_spirit"]="cvt"

#descriptors["Gomes_sdbc_walls_and_robots_std"]=10
#voronoi["Gomes_sdbc_walls_and_robots_std"]="cvt"

descriptors["history"]=3
voronoi["history"]=""

FitfunType="Foraging"
echo "simtime "${SimTime}
echo "FitfunType "${FitfunType}
perturbations_folder="experiments/harvesting/perturbations"
# for FaultType in "FAULT_PROXIMITYSENSORS_SETMIN" "FAULT_PROXIMITYSENSORS_SETMAX" "FAULT_PROXIMITYSENSORS_SETRANDOM" \
# "FAULT_ACTUATOR_LWHEEL_SETHALF" "FAULT_ACTUATOR_RWHEEL_SETHALF" "FAULT_ACTUATOR_BWHEELS_SETHALF"; do

declare -A faultnum

faultnum["sensor"]=30
faultnum["lwheel_set_half"]=1
faultnum["rwheel_set_half"]=1
faultnum["proximity_sensor"]=20
faultnum["ground_sensor"]=20
faultnum["actuator"]=20
faultnum["software"]=6      # number of agents  (1,0,0,0,0,0),(0,1,0,0,0,0), ...
faultnum["software_food"]=6 # number of agents  (1,0,0,0,0,0),(0,1,0,0,0,0), ...
faultnum["food_scarcity"]=1 # (will loop over food as a dummy)
faultnum["agents"]=12       # {1,2,...,12} agents included

for FaultCategory in proximity_sensor; do
    numfaults=${faultnum[${FaultCategory}]}
    echo "numfaults ${numfaults}"
    for FaultIndex in 4; do
        for key in ${!descriptors[@]}; do
            DescriptorType=${key}
            EffectiveDescriptorType=${DescriptorType}
            BD_DIMS=${descriptors[${key}]}
            CVT=${voronoi[${DescriptorType}]}
            if [ "$DescriptorType" = "baseline" ]; then
                tag=""
                SwarmBehaviour=${behav[${FitfunType}]}
                echo "SwarmBehaviour = "${SwarmBehaviour}
                sleep 5
            else
                tag=${CVT}${BD_DIMS}DANAREAL
                SwarmBehaviour="/"
                sleep 5
            fi
            echo "doing ${DescriptorType} now"
            echo "has ${BD_DIMS} dimensions"
            echo "tag is ${tag}"

            for Replicates in 1; do
                # Take template.argos and make an .argos file for this experiment
                SUFFIX=${Replicates}

                #read the data from the original experiment
                Base=${data}/${FitfunType}${large}/${DescriptorType}
                # look at archive dir at FAULT_NONE; config includes perturbations

                # look at archive dir at previous perturbation results; config is at FAULT_NONE
                FaultType="FILE:${perturbations_folder}/run${Replicates}_${FaultCategory}${fault_tag}p${FaultIndex}.txt"

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
                    food_loop="5"
                    FaultID=$(($FaultIndex - 1))
                    echo "food scarcity"
                elif [ "$FaultCategory" = "lwheel_set_half" ] || [ "$FaultCategory" = "rwheel_set_half" ]; then
                    robots=1
                    FaultID=-1
                    fault=$FaultType
                    echo "one wheel damaged"
                    EffectiveDescriptorType="empty"
                    echo "looking at perturbation: ${FaultType}"
                    sleep 4
                else
                    robots=6
                    fault=$FaultType
                    FaultID=-1
                    echo "other category (e.g., sensor or actuator)"
                fi

                robots=$(($scale * $robots))
                echo "fault ${fault}   robots ${robots}  FaultID  $FaultID SimTime $SimTime"
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

                    #write data to these folders
                    if [ "$2" = "best" ]; then
                        echo "will look for perturbations at run${Replicates}_${FaultCategory}p${FaultIndex}"
                        ConfigFolder=${Base}
                        ConfigFile=${ConfigFolder}/history_exp_${Replicates}_${FaultCategory}p${FaultIndex}.argos # just to write the history
                        ArchiveDir=${ConfigFolder}/faultyrun${Replicates}_${FaultCategory}p${FaultIndex}${food_tag}/
                        export Outfolder=${ArchiveDir}/${video}/results${SUFFIX}_${fault_tag} # where to output the results
                        export Searchfolder=${ArchiveDir}/results${SUFFIX}      # where to search for best indiv
                        FaultType="FILE:${perturbations_folder}/run${Replicates}_${FaultCategory}p${FaultIndex}.txt"
                    elif [ "$2" = "best_compare" ]; then # compare the best one but evaluate it in the no-fault environment
                        echo "will look for perturbations at run${Replicates}_${FaultCategory}p${FaultIndex}"
                        ConfigFolder=${Base}
                        ConfigFile=${ConfigFolder}/history_exp_${Replicates}_${FaultCategory}p${FaultIndex}.argos # just to write the history
                        ArchiveDir=${ConfigFolder}/faultyrun${Replicates}_${FaultCategory}p${FaultIndex}${food_tag}/
                        export Outfolder=${ArchiveDir}/${video}/results${SUFFIX}_${fault_tag} # where to output the results
                        export Searchfolder=${ArchiveDir}/results${SUFFIX}       # where to search for best indiv
                        FaultType="FAULT_NONE"
                    elif [ "$2" = "impact" ]; then # assess impact of fault on the normal individual
                        echo "will look for perturbations at run${Replicates}_${FaultCategory}p${FaultIndex}"
                        FaultType="FILE:${perturbations_folder}/run${Replicates}_${FaultCategory}${fault_tag}p${FaultIndex}.txt"
                        ConfigFolder=${Base}/faultyrun${Replicates}_p${FaultIndex}${food_tag}
                        mkdir -p ${ConfigFolder}
                        ConfigFile=${ConfigFolder}/exp_${SUFFIX}.argos
                        ArchiveDir=${ConfigFolder}/faultyrun${Replicates}_p${FaultIndex}/
                        export Outfolder=${ArchiveDir}/${video}/results${SUFFIX}_${fault_tag} # where to output the results
                        export Searchfolder=${Base}/FAULT_NONE/results${SUFFIX}  # where to search for best indiv (here search for the FAULT_NONE)
                    else
                        # look at archive dir at previous perturbation results; config is at FAULT_NONE
                        FaultType="FILE:${perturbations_folder}/run${Replicates}_${FaultCategory}${fault_tag}p${FaultIndex}.txt"
                        ConfigFolder=${Base}/faultyrun${Replicates}_${FaultCategory}p${FaultIndex}${food_tag}
                        mkdir -p ${ConfigFolder}
                        ConfigFile=${ConfigFolder}/exp_${SUFFIX}.argos
                        export ArchiveDir=${Base}/results${SUFFIX} # point to the generation file and archive
                        export archivefile="${ArchiveDir}/archive_${FINALGEN_ARCHIVE}.dat"
                        Outfolder=${ConfigFolder}/results${SUFFIX}_${fault_tag}
                    fi
                    mkdir -p $Outfolder
                    echo "config ${ConfigFile}"
                    touch ${Config0File}
                    sed -e "s|THREADS|0|" \
                        -e "s|TRIALS|8|" \
                        -e "s|ROBOTS|${robots}|" \
                        -e "s|EXPERIMENT_LENGTH|${SimTime}|" \
                        -e "s|SEED|${Replicates}|" \
                        -e "s|FITFUN_TYPE|${FitfunType}|" \
                        -e "s|DESCRIPTOR_TYPE|analysis|" \
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
                        ${TemplateFile} \
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
                    #bash zero_padding_data.sh ${Base}/results${SUFFIX} # make sure everything is zero-padded
                    if [ "$DescriptorType" = "baseline" ]; then
                        echo "will submit baseline job"
                        bash submit_baseline_job.sh # just record the performance of the baseline controllers in the faulty environment
                    else
                        if [ "${video}" = "video" ]; then
                            if [ "$2" = "best" ]; then
                                export VIDEOFILE="/home/david/Videos/${FitfunType}_${DescriptorType}_FAULT${FaultIndex}_run${Replicates}" #
                            elif [ "$2" = "best_compare" ]; then
                                export VIDEOFILE="/home/david/Videos/${FitfunType}_${DescriptorType}_FAULT${FaultIndex}_run${Replicates}_NORMAL" #
                            elif [ "$2" = "impact" ]; then
                                export VIDEOFILE="/home/david/Videos/${FitfunType}_${DescriptorType}_FAULT${FaultIndex}_run${Replicates}_IMPACT" #

                            fi
                            bash submit_test.sh "video"
                        elif [ "$2" = "best" ]; then
                            bash submit_test.sh $2 # submit in your own system; 7Zip support needed+jobs are short
                        else
                            echo "submit test"
                            bash submit_test.sh $2 # submit to iridis
                        fi
                    fi
                done
            done
        done
    done
done
