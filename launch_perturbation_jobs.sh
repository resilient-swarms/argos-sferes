#!/bin/bash -e



source activate py3.7 # just for the cvt initialisation 


data=experiments/datanew
export FINALGEN=1000
# Create a data diretory
mkdir -p $data
declare -A descriptors
declare -A voronoi


descriptors["history"]=2
descriptors["cvt_mutualinfoact"]=14
descriptors["cvt_mutualinfo"]=21
descriptors["cvt_spirit"]=400
descriptors["cvt_sdbc_all_std"]=14
voronoi["cvt_mutualinfo"]="cvt"
voronoi["history"]=""
voronoi["cvt_mutualinfoact"]="cvt"
voronoi["cvt_mutualinfo"]="cvt"
voronoi["cvt_spirit"]="cvt"
voronoi["cvt_sdbc_all_std"]="cvt"
for FaultType in "FAULT_NONE" "FAULT_PROXIMITYSENSORS_SETMIN" "FAULT_PROXIMITYSENSORS_SETMAX" "FAULT_PROXIMITYSENSORS_SETRANDOM" \
"FAULT_ACTUATOR_LWHEEL_SETHALF" "FAULT_ACTUATOR_RWHEEL_SETHALF" "FAULT_ACTUATOR_BWHEELS_SETHALF"; do
for FaultID in "0"; do
for FitfunType in Coverage; do
    echo 'Fitfun'${FitFunType}
    for SensorRange in 50; do
	echo 'sens'${SensorRange}
    for key in ${!descriptors[@]}; do
	DescriptorType=${key}
	BD_DIMS=${descriptors[${key}]}
        CVT=${voronoi[${DescriptorType}]}
	tag=${CVT}${BD_DIMS}D
	echo "doing ${DescriptorType} now"
	echo "has ${BD_DIMS} dimensions"
	echo "tag is ${tag}"
	mkdir -p $data/${FitfunType}/${DescriptorType}

	for Replicates in $(seq 1 5); do
                       
            # Take template.argos and make an .argos file for this experiment
            SUFFIX=${Replicates}
	    ArchiveFolder=${data}/${FitfunType}range${SensorRange}/${DescriptorType}/results${SUFFIX}
            ConfigFolder=${data}/${FitfunType}range${SensorRange}/${DescriptorType}/${FaultType}${FaultID}
            Centroidsfolder=${data}/${FitfunType}range${SensorRange}/${DescriptorType}/results${SUFFIX}
            OutputFolder=${ConfigFolder}/${Replicates}
            rm -rf ${OutputFolder} # remove if it already exists
            mkdir -p ${OutputFolder}
	    ConfigFile=${ConfigFolder}/exp_${SUFFIX}.argos
           
	    touch ${ConfigFile} 
	   sed -e "s|TRIALS|15|" \
                -e "s|ROBOTS|1|"                    \
                -e "s|SEED|${Replicates}|"                    \
                -e "s|FITFUN_TYPE|${FitfunType}|"                   \
                -e "s|DESCRIPTOR_TYPE|${DescriptorType}|"                  \
                -e "s|OUTPUTFOLDER|${OutputFolder}|" \
                -e "s|CENTROIDSFOLDER|${Centroidsfolder}|" \
		        -e "s|SENSOR_RANGE|${SensorRange}|" \
		        -e "s|NOISE_LEVEL|0.05|"    \
                -e "s|BEHAVIOUR_TAG|${tag}|" \
                -e "s|FAULT_TYPE|${FaultType}|" \
                -e "s|FAULT_ID|${FaultID}|"\
		experiments/experiment_template_perturbation.argos \
                > ${ConfigFile}
             if [ ! -z "${CVT}"  ]; then
        	echo ${CVT}
		#python sferes2/modules/cvt_map_elites/cvt.py -k 1000 -d ${BD_DIMS} -p 100000 -f ${Outfolder}
		#vsuffix='-v '${CVT}
		#cho ${cvsuffix}
	     fi
	  
	   # Call ARGoS
	   export BD=${BD_DIMS}
	   export CONFIG=${ConfigFile}
	   export OUTPUTDIR=${OutputFolder}
	   export ARCHIVEDIR=${ArchiveFolder}
	   export VORONOI=${CVT}
         sbatch submit_test.sh 
        done
	done
    done
done
done
done

