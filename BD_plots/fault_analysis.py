
import os
HOME_DIR = os.environ["HOME"]
fault_dir=HOME_DIR+"/argos-sferes/experiments/perturbations"
from perturbation_analysis import *
def get_fault_proportion(fault_type,run,fault_index):
    f = open(fault_dir+"/run"+run+"_p"+fault_index+".txt", 'r')
    lines = f.read().split(',')
    f.close()
    num=0
    for line in lines:
        if fault_type=="Proxi":
            if "PROXIMITYSENSORS" in line:
                num+=1
        elif fault_type=="RAB":
            if "RABSENSOR" in line:
                num+=1

        elif fault_type=="Act":
            if "ACTUATOR" in line:
                num+=1
        else:
            raise Exception("choose either Proxi, RAB, or Act as a fault")

    return num/float(len(lines))

def get_all_fault_proportions(fault_type):
    """
    :param fault_type the type of fault
    :return proportion of this type in the swarm for each faulty environment
    """
    proportions=[]
    for j in range(len(fitfuns)):
        print(fitfuns[j])
        for fault in range(len(faults)):   # same order of gathering as the impact in significance_data function in perturbation_analysis.py
            for run in runs:
                proportions.append(get_fault_proportion(fault_type,str(run),str(fault)))

    return proportions






if __name__ == "__main__":
    proxi_proportions = get_all_fault_proportions("Proxi")



    rab_proportions = get_all_fault_proportions("RAB")

    act_proportions = get_all_fault_proportions("Act")