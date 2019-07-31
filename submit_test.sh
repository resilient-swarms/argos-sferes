#!/bin/bash
#SBATCH --ntasks-per-node=1    # Tasks per node
#SBATCH --nodes=1                # Number of nodes requested
#SBATCH --time=24:00:00          # walltime
#SBATCH --mem=10G
#SBATCH --job-name=map_evaluation

source activate py3.7 # just for the cvt initialisation

generationfile="${ARCHIVEDIR}/gen_${FINALGEN_GENFILE}"
archivefile="${ARCHIVEDIR}/archive_${FINALGEN_ARCHIVE}.dat"

jobtocome="${COMMAND} ${CONFIG} -d ${OUTPUTDIR} --load ${generationfile} --o outputfile"
echo "Starting the following command: "${jobtocome}" for all individuals"
echo "Looking for individuals at: "${archivefile}

python BD_plots/process_archive_data.py -c "${jobtocome}" -p "${archivefile}" -o "${OUTPUTDIR}"
