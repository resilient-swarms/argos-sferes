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
softwarefood_perturbations = ["software_foodp" + str(i) + "f" + str(f) for i in range(1, software_perturbs + 1) for
                              f in range(1, num_food + 1)]
foodscarcity_perturbations = ["food_scarcityp1f"+str(f) for f in range(1, 7)]
agent_perturbations = ["agentsp" + str(i) for i in range(1, max_num_agents + 1) if i!=6]
foraging_perturbations = proximity_sensor_perturbations + ground_sensor_perturbations + actuator_perturbations + \
                         software_perturbations + softwarefood_perturbations + foodscarcity_perturbations  + agent_perturbations


num_fault_types=7

foraging_fault_types=["Proximity-Sensor","Ground-Sensor","Actuator","Software-Nest","Software-Food","Food-Scarcity","Agents"]

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
    elif fault.startswith("agentsp"):
        return "Agents",6
    else:
        Exception("unknown faulttype")