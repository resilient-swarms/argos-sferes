
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


SENSOR_PERTURBATIONS=["FAULT_NONE","FAULT_PROXIMITYSENSORS_SETMIN", "FAULT_PROXIMITYSENSORS_SETMAX", "FAULT_PROXIMITYSENSORS_SETRANDOM",
                      "FAULT_GROUNDSENSORS_SETMIN", "FAULT_GROUNDSENSORS_SETMAX", "FAULT_GROUNDSENSORS_SETRANDOM"]
ACT_PERTURBATIONS=["FAULT_NONE","FAULT_ACTUATOR_LWHEEL_SETHALF", "FAULT_ACTUATOR_RWHEEL_SETHALF", "FAULT_ACTUATOR_BWHEELS_SETHALF"]
SOFTWARE_FAULTS=[]


def software_faults(num_agents, filename):
    for i in range(num_agents):
        with open(filename+"p"+str(i)+".txt","w+") as f:
            combined_perturbations=["FAULT_NONE" for i in range(num_agents)]
            combined_perturbations[i] = "SOFTWARE_FAULT"
            string_p=""
            for j in range(len(combined_perturbations) - 1):
                string_p+=combined_perturbations[j]+","
            string_p += combined_perturbations[-1]
            f.write(string_p)


def random_combinations(N,num_agents):
    return np.random.randint(0,N,num_agents)

def write_superset(filename,basic_perturbations,num_agents,elements):
    N = len(basic_perturbations)
    for i in elements:
        with open(filename+"p"+str(i)+".txt","w+") as f:
            indexes = random_combinations(N,num_agents)
            combined_perturbations=""
            for j in range(len(indexes) - 1):
                ind = indexes[j]
                combined_perturbations+=basic_perturbations[ind]+","
            combined_perturbations += basic_perturbations[indexes[-1]]
            f.write(combined_perturbations)


if __name__ == "__main__":
    # for run in range(1,6):
    #     write_superset(home+"/argos-sferes/experiments/perturbations/run"+str(run)+"_",BASIC_PERTURBATIONS,10,range(0,100))

    num_agents=6
    for run in range(1, 6):
        write_superset(home + "/argos-sferes/experiments/harvesting/perturbations/run" + str(run) + "_sensor", SENSOR_PERTURBATIONS, num_agents,
                       range(0, 30))
        write_superset(home + "/argos-sferes/experiments/harvesting/perturbations/run" + str(run) + "_actuator", ACT_PERTURBATIONS, num_agents,
                       range(0, 20))

        software_faults(num_agents,home + "/argos-sferes/experiments/harvesting/perturbations/run" + str(run) + "_software")



