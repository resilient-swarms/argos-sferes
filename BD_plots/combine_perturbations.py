
"""
combine basic perturbations for different agents
"""

import numpy as np
import pickle
import os

home=os.environ["HOME"]

BASIC_PERTURBATIONS=["FAULT_NONE","FAULT_PROXIMITYSENSORS_SETMIN", "FAULT_PROXIMITYSENSORS_SETMAX", "FAULT_PROXIMITYSENSORS_SETRANDOM",
"FAULT_ACTUATOR_LWHEEL_SETHALF", "FAULT_ACTUATOR_RWHEEL_SETHALF", "FAULT_ACTUATOR_BWHEELS_SETHALF",
                     "FAULT_RABSENSOR_SETOFFSET"]

N =len(BASIC_PERTURBATIONS)


def random_combinations(num_agents):
    return np.random.randint(0,N,num_agents)

def write_superset(filename,basic_perturbations,num_agents,elements):
    for i in elements:
        with open(filename+"p"+str(i)+".txt","w+") as f:
            indexes = random_combinations(num_agents)
            combined_perturbations=""
            for j in range(len(indexes) - 1):
                ind = indexes[j]
                combined_perturbations+=basic_perturbations[ind]+","
            combined_perturbations += basic_perturbations[indexes[-1]]
            f.write(combined_perturbations)


if __name__ == "__main__":
    for run in range(1,6):
        write_superset(home+"/argos-sferes/experiments/perturbations/run"+str(run)+"_",BASIC_PERTURBATIONS,10,range(40,100))


