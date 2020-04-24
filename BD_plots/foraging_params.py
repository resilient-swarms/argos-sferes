import os
import pickle
NUM_SECONDS=120.0
TICKS_PER_SECOND=5.0
TICKS_PER_TRIAL=NUM_SECONDS*TICKS_PER_SECOND

baseline_performances = pickle.load(open("data/fitfun/foraging_maximal_fitness.pkl", "rb"))


runs=range(1,6)
bd_type = ["history"]  # legend label
legend_labels = ["HBD"]  # labels for the legend
fitfuns = ["Foraging"]
fitfunlabels = [""]


colors = ["C" + str(i) for i in range(len(bd_type))]
markers = [(2, 1, 0), (3, 1, 0), (2, 1, 1), (3, 1, 1)]

HOME_DIR = os.environ["HOME"]
datadir = HOME_DIR + "/Data/"
generation = "20000"
history_type = "xy"




sensor_perturbs = 20
actuator_perturbs = 20
software_perturbs = 6
num_food = 5
max_num_agents = 11

#sensor_perturbations = ["sensorp" + str(i) for i in range(1, sensor_perturbs + 1)]
proximity_sensor_perturbations = ["proximity_sensorp" + str(i) for i in range(1, sensor_perturbs + 1)]
ground_sensor_perturbations = ["ground_sensorp" + str(i) for i in range(1, sensor_perturbs + 1)]
actuator_perturbations = ["actuatorp" + str(i) for i in range(1, actuator_perturbs + 1)]
software_perturbations = ["softwarep" + str(i) for i in range(1, software_perturbs + 1)]
softwarefood_perturbations = ["software_foodp" + str(i) + "f" + str(1) for i in range(1, software_perturbs + 1)]
foodscarcity_perturbations = ["food_scarcityp1f"+str(f) for f in range(1, 7)]
agent_perturbations = ["agentsp" + str(i) for i in [3,12,24]]
foraging_perturbations = proximity_sensor_perturbations + ground_sensor_perturbations + actuator_perturbations + \
                         software_perturbations + softwarefood_perturbations + foodscarcity_perturbations  + agent_perturbations


num_fault_types=9

foraging_fault_types=["Proximity-Sensor","Ground-Sensor","Actuator","Software-Nest","Software-Food","Food-Scarcity",
                      "3 agents","12 agents", "24 agents"]

def get_fault_type(fault):
    if fault.startswith("proximity_sensorp"):
        return "Proximity-Sensor",0
    elif fault.startswith("ground_sensorp"):
        return "Ground-Sensor",1
    elif fault.startswith("actuatorp"):
        return "Actuor",2
    elif fault.startswith("softwarep"):
        return "Software-Nest",3
    elif fault.startswith("software_foodp"):
        return "Software-Food",4
    elif fault.startswith("food_scarcityp"):
        return "Food-Scarcity",5
    elif fault.startswith("agentsp3"):
        return "Agents",6
    elif fault.startswith("agentsp12"):
        return "Agents",7
    elif fault.startswith("agentsp24"):
        return "Agents",8
    else:
        Exception("unknown faulttype")