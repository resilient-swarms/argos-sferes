#!/bin/bash
#SBATCH --ntasks-per-node=1    # Tasks per node
#SBATCH --nodes=1                # Number of nodes requested
#SBATCH --time=00:30:00          # walltime
#SBATCH --mem=10G
#SBATCH --job-name=obsavoid_evol${VORONOI}${BD}D


FILE=./bin/obsavoid_evol${VORONOI}${BD}D
if [ -f "$FILE" ]; then
    echo "$FILE exist"
else 
    echo "$FILE does not exist"
    exit 1;
fi
generationfile="${ARCHIVEDIR}/gen_${FINALGEN}"
archivefile="${ARCHIVEDIR}/archive_${FINALGEN}.dat"

jobtocome="${FILE} ${CONFIG} -d ${OUTPUTDIR} --load ${generationfile} --o outputfile"
echo "Starting the following command: "${jobtocome}" for all individuals"
echo "Looking for individuals at: "${archivefile}

python process_archive_data.py -c "${jobtocome}" -p "${archivefile}"
