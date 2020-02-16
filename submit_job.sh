#!/bin/bash

#SBATCH --exclusive   #Tasks per node
#SBATCH --nodes=1                # Number of nodes requested
#SBATCH --time=60:00:00         # walltime
#SBATCH --mem-per-cpu=10G  # actually no need to specify when nodes > 20
#SBATCH --job-name=parallel


FILE=./bin/behaviour_evol${tag}

jobtocome="${FILE} ${CONFIG} -d ${OUTPUTDIR}"
if [ ! -z  "${GENERATION_FILE}" ]; then
   echo "Generation file already exists; plan to resume"
   jobtocome="${jobtocome} --resume ${GENERATION_FILE}"
fi

echo "Starting the following command: "${jobtocome}
ulimit -c 0
${jobtocome}
