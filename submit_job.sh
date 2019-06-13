#!/bin/bash

#SBATCH --ntasks-per-node=1    # Tasks per node
#SBATCH --nodes=1                # Number of nodes requested
#SBATCH --time=60:00:00          # walltime
#SBATCH --mem=20G
#SBATCH --job-name=behaviour_evol


FILE=./bin/behaviour_evol${VORONOI}${BD}D
if [ -f "$FILE" ]; then
    echo "$FILE exist"
else 
    echo "$FILE does not exist"
    exit 1;
fi
jobtocome="${FILE} ${CONFIG} -d ${OUTPUTDIR}"
if [ ! -z  "${GENERATION_FILE}" ]; then
   echo "Generation file already exists; plan to resume"
   jobtocome="${jobtocome} --resume ${GENERATION_FILE}"
fi

echo "Starting the following command: "${jobtocome}
${jobtocome}
