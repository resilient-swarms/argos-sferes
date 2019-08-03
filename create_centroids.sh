#!/bin/bash

#SBATCH --ntasks-per-node=1  #Tasks per node
#SBATCH --nodes=1                # Number of nodes requested
#SBATCH --time=02:00:00         # walltime
#SBATCH --mem-per-cpu=80G  # actually no need to specify when nodes > 20

source activate py3.7

cd sferes2/modules/cvt_map_elites
python cvt.py -d $1 -p 100000 -k 4096 -f /home/dmb1m19/argos-sferes/experiments/centroids
