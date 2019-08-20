#!/bin/bash

#SBATCH --exclusive   #Tasks per node
#SBATCH --nodes=1                # Number of nodes requested
#SBATCH --time=60:00:00         # walltime
#SBATCH --mem-per-cpu=10G  # actually no need to specify when nodes > 20
#SBATCH --job-name=parallel


FILE=./bin/baseline_behaviour

jobtocome="${FILE} ${CONFIG} -d ${OUTPUTDIR}"
echo "Starting the following command: "${jobtocome}
${jobtocome}
