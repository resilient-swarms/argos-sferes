#!/bin/bash
#SBATCH --ntasks-per-node=1    # Tasks per node
#SBATCH --nodes=1                # Number of nodes requested
#SBATCH --time=18:00:00          # walltime
#SBATCH --mem=10G
#SBATCH --job-name=map_evaluation


jobtocome="${COMMAND} ${ConfigFile} ${ArchiveFolder} ${Generation} ${BO_OutputFolder} -d ${BO_OutputFolder} --load \
        ${ArchiveFolder}/gen_${Generation} -n 2713 -o outputfile"
      

echo "Starting the following command: "${jobtocome}
ulimit -c 0
${jobtocome}
