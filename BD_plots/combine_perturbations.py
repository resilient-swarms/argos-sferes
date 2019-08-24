
"""
combine basic perturbations for different agents
"""

import numpy as np
import pickle


BASIC_PERTURBATIONS=["FAULT_NONE","FAULT_PROXIMITYSENSORS_SETMIN", "FAULT_PROXIMITYSENSORS_SETMAX", "FAULT_PROXIMITYSENSORS_SETRANDOM",
"FAULT_ACTUATOR_LWHEEL_SETHALF", "FAULT_ACTUATOR_RWHEEL_SETHALF", "FAULT_ACTUATOR_BWHEELS_SETHALF"]


def random_combinations(num_agents):
    return np.random.randint(0,7,num_agents)

def write_superset(filename,basic_perturbations,num_agents,num_elements):
    superset=[]
    for i in range(num_elements):
        indexes = random_combinations(num_agents)
        combined_perturbations=[basic_perturbations[i] for i in indexes]
        superset.append(combined_perturbations)

    pickle.dump(filename)


if __name__ == "__main__":
   write_superset("",BASIC_PERTURBATIONS,10,100)