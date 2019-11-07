"""

some plots to judge the efficiency of IT&E

"""
from perturbance_metrics import *
import matplotlib.pyplot as plt
import numpy as np

DATAPATH="/home/david/Data"
GEN="30000"

def get_BO_performance(filename):
    parsed_file_list = read_spacedelimited(filename)
    best_performance = float(parsed_file_list[-1][-1]) # last column of the last line in the file
    number_of_tries = len(parsed_file_list) - 1 # length -1 as the number of function evaluations
    return best_performance, number_of_tries



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
    faultperformances=[]
    recoveryperformances=[]
    recoveryBOperformances = []
    evaluations=[]
    for f, fault in enumerate(faults):
        for i,run in enumerate(runs):
            # transfer the original individual to the faulty environment
            faultpath = DATAPATH + "/" + fitfun + "range0.11/" + descriptor + "/faultyrun" + str(run) + "_p" + str(f) + "/results" + str(run) + "/analysis" + GEN + "_handcrafted.dat"
            temp = np.array(list(get_ind_performances_uniquearchive(faultpath).values())).flatten()
            fperformance = temp[best_individuals[i]]
            faultperformances.append(fperformance)

            # obtain the best solution in the faulty environment
            faultypath = DATAPATH + "/" + fitfun + "range0.11/" + descriptor + "/faultyrun" + str(run) + "_p" + str(
                f) + "/results" + str(run) + "/analysis" + GEN + "_handcrafted.dat"
            best_individual, best_performance = get_best_individual(faultypath, add_performance=True, index_based=True)
            recoveryperformances.append(best_performance)

            # obtain the best solution from BO in the faulty environment
            BOpath = DATAPATH + "/" + fitfun + "range0.11/" + descriptor + "/faultyrun" + str(run) + "_p" + str(
                f) + "/BOresults" + str(
                run) + "/BO_output/best_observations.dat"
            performance, tries = get_BO_performance(BOpath)
            recoveryBOperformances.append(performance)
            evaluations.append(tries)

    print(evaluations)

    # Calculate the average
    nofault_mean = np.mean(nofaultperformances)
    fault_mean = np.mean(faultperformances)
    recovery_mean = np.mean(recoveryperformances)
    recoveryBO_mean = np.mean(recoveryBOperformances)


    # Calculate the standard deviation
    nofault_std = np.std(nofaultperformances)
    fault_std = np.std(faultperformances)
    recovery_std = np.std(recoveryperformances)
    recoveryBO_std= np.std(recoveryBOperformances)


    # Create lists for the plot
    scenarios = ['normal', 'fault', 'recovery','recovery-BO']
    x_pos = np.arange(len(scenarios))
    y = [nofault_mean, fault_mean, recovery_mean,recoveryBO_mean]
    error = [nofault_std, fault_std, recovery_std,recoveryBO_std]

    # Build the plot
    fig, ax = plt.subplots()
    ax.bar(x_pos, y, yerr=error, align='center', alpha=0.5, ecolor='black', capsize=10)
    ax.set_ylabel('fitness')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(scenarios)
    #ax.set_title('')
    ax.yaxis.grid(True)

    # Save the figure and show
    plt.tight_layout()
    plt.savefig('bar_plot_with_error_bars.png')
    plt.show()



if __name__ == "__main__":
    impact_of_fault(fitfun="Aggregation", descriptor="history", runs=[1,2,3,4,5], faults=[0])

