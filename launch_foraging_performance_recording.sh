#!/bin/bash -e

# run environment_diversity on generation START:END by STEP

data=$1
# 2 is all vs best
video=$3
UseVirtual=$4
if [ "$UseVirtual" = "True" ]; then
    VirtualFolder="virtual_energy_exp"
fi

SimTime=120

echo "doing generation ${FINALGEN_ARCHIVE}"
sleep 2.5

FitfunType="Foraging"
echo "simtime "${SimTime}
echo "FitfunType "${FitfunType}
perturbations_folder="experiments/harvesting/perturbations"

# Create a data diretory
mkdir -p $data
declare -A descriptors
declare -A voronoi
declare -A behav



MINGEN=20000 #
MAXGEN=20000
STEP=500


command="bin/analysis" # note: cvt and 10D does not really matter since we are not evolving

#descriptors["Gomes_sdbc_walls_and_robots_std"]=10
#voronoi["Gomes_sdbc_walls_and_robots_std"]="cvt"

# descriptors["environment_diversity"]=6
# voronoi["environment_diversity"]=""

descriptors["history"]=3
voronoi["history"]=""

#descriptors["baseline"]=""
#voronoi["baseline"]=""

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
        tag=${CVT}${BD_DIMS}DANAREAL
        SwarmBehaviour="/"
    fi
    echo "doing ${DescriptorType} now"
    echo "has ${BD_DIMS} dimensions"
    echo "tag is ${tag}"

    for Replicates in $(seq 1 5); do
        for GEN in $(seq ${MINGEN} ${STEP} ${MAXGEN}); do
            # Take template.argos and make an .argos file for this experiment
            SUFFIX=${Replicates}

            #read the data from the original experiment
            Base=${data}/Foraging/${DescriptorType}
            ArchiveDir=${Base}/results${SUFFIX} # point to the generation file and archive

            #write data to these folders
            ConfigFolder=${Base}/${FaultType}/${video}
            ConfigFile=${ConfigFolder}/exp_${SUFFIX}.argos
            Outfolder=${ConfigFolder}/results${SUFFIX}                # where to drop the results
            export Searchfolder=${Base}/${FaultType}/results${SUFFIX} # where to search for best indiv

            mkdir -p $Outfolder
            echo "config ${ConfigFile}"
            touch ${ConfigFile}

            mkdir -p ${ConfigFolder}
            ConfigFile=${ConfigFolder}/exp_${SUFFIX}.argos
            export ArchiveDir=${Base}/results${SUFFIX} # point to the generation file and archive
            export archivefile="${ArchiveDir}/archive_${FINALGEN_ARCHIVE}.dat"
            Outfolder=${ConfigFolder}/results${SUFFIX}/${VirtualFolder}
            echo "Outfolder ${Outfolder}"
            sleep 3
            rm ${Outfolder}/fitness
            mkdir -p $Outfolder
            echo "config ${ConfigFile}"
            touch ${ConfigFile}
            sed -e "s|THREADS|0|" \
                -e "s|TRIALS|8|" \
                -e "s|ROBOTS|6|" \
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
                -e "s|FAULT_TYPE|FAULT_NONE|" \
                -e "s|FAULT_ID|-1|" \
                -e "s|FOOD_ID|-1|" \
                -e "s|SWARM_BEHAV|${SwarmBehaviour}|" \
                -e "s|USE_VIRTUAL|${UseVirtual}|" \
                experiments/harvesting/harvesting_template.argos \
                >${ConfigFile}

            if [ ! -z "${CVT}" ]; then
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

            export FINALGEN_ARCHIVE=${GEN} # never forget zero-padding for generation file, not for archive file
            if [ ${GEN} -gt 9999 ]; then
                export FINALGEN_GENFILE=${GEN} # add one zero
            elif [ ${GEN} -gt 999 ]; then
                export FINALGEN_GENFILE=0${GEN} # add two zeros
            elif [ ${GEN} -gt 99 ]; then
                export FINALGEN_GENFILE=00${GEN} # add two zeros
            elif [ ${GEN} -gt 9 ]; then
                export FINALGEN_GENFILE=000${GEN} # add two zeros
            else
                export FINALGEN_GENFILE=0000${GEN} # add two zeros
            fi
            export generationfile="${Base}/results${SUFFIX}/gen_${FINALGEN_GENFILE}"
            export archivefile="${ArchiveDir}/archive_${FINALGEN_ARCHIVE}.dat"

            echo "submitting job"
            bash zero_padding_data.sh ${Base}/results${SUFFIX} # make sure everything is zero-padded
            if [ "$DescriptorType" = "baseline" ]; then
                echo "will submit baseline job"
                bash submit_baseline_job.sh # just record the performance of the baseline controllers in the faulty environment
            else
                if [ "${video}" = "video" ]; then

                    export VIDEOFILE="/home/david/Videos/${FitfunType}_${DescriptorType}_NOFAULT_run${Replicates}" #
                    echo "exporting video file ${VIDEOFILE}"
                    echo "submitting video-test"
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
