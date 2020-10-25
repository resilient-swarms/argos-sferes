

from foraging.foraging_params import *
from process_archive_data import *
from combine_perturbations import *

def analyse_faults(max_eval=30):
    def get_fault_string(folder,run,fault):
        return read_commadelimited(folder+"run"+str(run)+"_"+fault+".txt")[0]

    def get_performance_folder(run, fault):
        BD_dir = datadir + "/ForagingLarge/history/"
        return BD_dir + "faultyrun" + str(run) + "_" + fault+"/results"+str(run)+"/single_exp/BO_outputnoID/"
    folder = "/home/david/argos-sferes/experiments/harvesting/perturbations/"
    # proximity sensor faults

    best_prox_performances={string:[] for string in PROXIMITY_SENSOR_PERTURBATIONS}
    for fault in proximity_sensor_perturbations:
        for run in runs:
            # get the fault
            faultstring = get_fault_string(folder,run,fault)
            #print(faultstring)
            # get the best performance files
            performancefolder = get_performance_folder(run, fault)
            for robot in range(int(NUM_AGENTS)):
                lines = read_spacedelimited(performancefolder+"async_stats_best"+str(robot)+".dat")
                best_prox_performances[faultstring[robot]].append(float(lines[max_eval][-1]))

    for key, val in best_prox_performances.items():
        print(key,": ",np.mean(val)," +/- ",np.std(val))


    best_ground_performances={string:[] for string in GROUND_SENSOR_PERTURBATIONS}
    for fault in ground_sensor_perturbations:
        for run in runs:
            # get the fault
            faultstring = get_fault_string(folder,run,fault)
            #print(faultstring)
            # get the best performance files
            performancefolder = get_performance_folder(run, fault)
            for robot in range(int(NUM_AGENTS)):
                lines = read_spacedelimited(performancefolder+"async_stats_best"+str(robot)+".dat")
                best_ground_performances[faultstring[robot]].append(float(lines[max_eval][-1]))

    for key, val in best_ground_performances.items():
        print(key,": ",np.mean(val)," +/- ",np.std(val))
    # actuator faults
    best_actuator_performances={string:[] for string in ACT_PERTURBATIONS}
    for fault in actuator_perturbations:
        for run in runs:
            # get the fault
            faultstring = get_fault_string(folder,run,fault)
            #print(faultstring)
            # get the best performance files
            performancefolder = get_performance_folder(run, fault)
            for robot in range(int(NUM_AGENTS)):
                lines = read_spacedelimited(performancefolder+"async_stats_best"+str(robot)+".dat")
                best_actuator_performances[faultstring[robot]].append(float(lines[max_eval][-1]))

    for key, val in best_actuator_performances.items():
        print(key,": ",np.mean(val)," +/- ",np.std(val))