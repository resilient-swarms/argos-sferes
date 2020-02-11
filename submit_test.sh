#!/bin/bash
#SBATCH --ntasks-per-node=1    # Tasks per node
#SBATCH --nodes=1                # Number of nodes requested
#SBATCH --time=18:00:00          # walltime
#SBATCH --mem=10G
#SBATCH --job-name=map_evaluation

source activate py3.7 # just for the cvt initialisation



echo ${CONFIG}
sleep 5


if [ "$1" != "all" ]; then
    # now get the history of the best solution
    jobtocome="${COMMAND} ${CONFIG} ${1} ${FINALGEN_ARCHIVE} -d ${OUTPUTDIR} --load ${generationfile} --o outputfile"
    echo "Getting the ${1}:\nStarting the following command: "${jobtocome}""
    echo 
    echo "Looking for individuals at: "${archivefile}
    python BD_plots/process_archive_data.py -c "${jobtocome}" -o "${Searchfolder}" -g "${FINALGEN_ARCHIVE}" -b "best"
    if [ "${1}" = "video" ]; then
        echo "compiling video"
        bash compile_video.sh ${VIDEOFILE}
        echo "video compiled . Done."
    fi
    exit 1
fi 


# first get all the fitness and descriptor values
jobtocome="${COMMAND} ${CONFIG} all ${FINALGEN_ARCHIVE} -d ${OUTPUTDIR} --load ${generationfile} --o outputfile"
echo "Starting the following command: "${jobtocome}" for all individuals"
echo 
echo "Looking for individuals at: "${archivefile}
echo
echo "python BD_plots/process_archive_data.py -c "${jobtocome}" -p "${archivefile}" -o "${OUTPUTDIR}" -b "all""
python BD_plots/process_archive_data.py -c "${jobtocome}" -p "${archivefile}" -o "${OUTPUTDIR}" -b "all"


