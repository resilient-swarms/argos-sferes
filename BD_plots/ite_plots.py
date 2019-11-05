"""

some plots to judge the efficiency of IT&E

"""
from perturbance_metrics import *
import matplotlib.pyplot as plt
import numpy as np

DATAPATH="/home/david/Data"
GEN="30000"

def get_last_performance(filename):
    parsed_file_list = read_spacedelimited(filename)
    return parsed_file_list[-1][-1] # last column of the last line in the file



def impact_of_fault(fitfun,descriptor,runs,faults):
    """
    show
    1. the best performance in normal environment
    2. the best performance in the faulty environment
    3. the recovered solution based on IT&E
    :return:
    """
    nofaultperformances=[]
    best_individuals=[]
    for run in runs:
        nofaultpath=DATAPATH+"/"+fitfun+"range0.11/"+descriptor+"/FAULT_NONE/results"+str(run)+"/analysis"+GEN+"_handcrafted.dat"
        best_individual, best_performance = get_best_individual(nofaultpath, add_performance=True, index_based=True)
        nofaultperformances.append(best_performance)
        best_individuals.append(best_individual)

    # now create data for each fault
    faultperformances={}
    recoveryperformances={}
    for f, fault in enumerate(faults):
        faultperformances[fault]=[]
        recoveryperformances[fault] = []
        for i,run in enumerate(runs):
            # transfer the original individual to the faulty environment
            faultpath = DATAPATH + "/" + fitfun + "range0.11/" + descriptor + "/faultyrun" + str(run) + "_p" + str(f) + "/results" + str(run) + "/analysis" + GEN + "_handcrafted.dat"
            temp = np.array(list(get_ind_performances_uniquearchive(faultpath).values())).flatten()
            fperformance = temp[best_individuals[i]]
            faultperformances[fault].append(fperformance)

            # obtain the best solution from BO in the faulty environment
            BOpath = DATAPATH + "/" + fitfun + "range0.11/" + descriptor + "/faultyrun" + str(run) + "_p" + str(
                f) + "/BOresults" + str(
                run) + "/BO_output/best_observations.dat"
            recoveryperformances[fault].append(get_last_performance(BOpath))

    print()





if __name__ == "__main__":
    impact_of_fault(fitfun="Aggregation", descriptor="history", runs=[1], faults=[0])

