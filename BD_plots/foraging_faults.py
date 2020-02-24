sensor_perturbs = 30
actuator_perturbs = 20
software_perturbs = 6
num_food = 5
max_num_agents = 11

sensor_perturbations = ["sensorp" + str(i) for i in range(1, sensor_perturbs + 1)]
actuator_perturbations = ["actuatorp" + str(i) for i in range(1, actuator_perturbs + 1)]
software_perturbations = ["softwarep" + str(i) for i in range(1, software_perturbs + 1)]
softwarefood_perturbations = ["software_foodp" + str(i) + "f" + str(f) for i in range(1, software_perturbs + 1) for
                              f in range(1, num_food + 1)]
agent_perturbations = ["agentsp" + str(i) for i in range(1, max_num_agents + 1) if i!=6]
foraging_perturbations = sensor_perturbations + actuator_perturbations + software_perturbations + softwarefood_perturbations + agent_perturbations


num_fault_types=5

foraging_fault_types=["Sensor","Actuator","Software-Nest","Software-Food","Agents"]

def get_fault_type(fault):
    if fault.startswith("sensorp"):
        return "Sensor",0
    elif fault.startswith("actuatorp"):
        return "Actuor",1
    elif fault.startswith("softwarep"):
        return "Software-Nest",2
    elif fault.startswith("software_foodp"):
        return "Software-Food",3
    elif fault.startswith("agentsp"):
        return "Agents",4
    else:
        Exception("unknown faulttype")