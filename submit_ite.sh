#!/bin/bash
#SBATCH --ntasks-per-node=1    # Tasks per node
#SBATCH --nodes=1                # Number of nodes requested
#SBATCH --time=18:00:00          # walltime
#SBATCH --mem=10G
#SBATCH --job-name=map_evaluation

source activate py3.7 # just for the cvt initialisation


fullcommand="${COMMAND} -f ${BO_OutputFolder} -m ${ArchiveFolder}  ${Generation} -e ${BO_Executable} ${ConfigFile} ${BaselineChoice}"
echo "will now execute ${fullcommand}"

sleep 5

# now execute the command
${fullcommand}






