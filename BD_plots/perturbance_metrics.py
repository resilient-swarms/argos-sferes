import numpy as np
from process_archive_data import *

from matplotlib import pyplot as plt

HOME_DIR = os.environ["HOME"]
RESULTSFOLDER="results"
FINAL_GEN=1200

def read_floats_from_file(file_in):
    x=[]
    for line in file_in.readlines():
        x.append(float(line))
    return x

# def make_boxplot(data,row_conditions,col_conditions,save_filename,xlabs):
#     # Creates four polar axes, and accesses them through the returned array
#     n_cols = len(col_conditions)
#     n_rows = len(row_conditions)
#     fig, axes = plt.subplots(n_rows, n_cols)
#     for i in range(n_rows):
#         for j in range(n_cols):
#             y = data[row_conditions[i]][col_conditions[j]]
#             axes[i,j].boxplot(y)
#             axes[i,j].set_xlabel(xlabs[j],rotation=0, fontsize=8)
#             axes[i,j].set_ylim([0,1])
#     fig.tight_layout()
#     fig.savefig(save_filename)
def make_boxplot(data,row_conditions,col_conditions,save_filename,xlabs,ylab):
    # Creates four polar axes, and accesses them through the returned array
    n_cols = len(col_conditions)
    n_rows = len(row_conditions)

    for i in range(n_rows):
        fig, axes = plt.subplots(1,1)
        y = list(data[row_conditions[i]].values())

        axes.boxplot(y)
        axes.set_xticklabels(xlabs,rotation=0, fontsize=8)
        axes.set_ylim([0,1])
        axes.set_ylabel(ylab)
        fig.suptitle(row_conditions[i])
        fig.tight_layout()
        fig.savefig(RESULTSFOLDER+"/"+row_conditions[i]+save_filename)



def make_barplot(data,row_conditions,col_conditions,save_filename,xlabs,ylab):
    # Creates four polar axes, and accesses them through the returned array
    n_cols = len(col_conditions)
    n_rows = len(row_conditions)

    for i in range(n_rows):
        fig, axes = plt.subplots(1,1)
        x = 3*np.array(range(1,len(xlabs)+1))
        y = list(data[row_conditions[i]].values())
        bar_width=1.0
        axes.bar(x, y,bar_width)
        plt.xticks(x, xlabs, fontsize=8)
        axes.set_ylim([0,1])
        axes.set_ylabel(ylab)
        fig.suptitle(row_conditions[i])
        fig.tight_layout()
        fig.savefig(RESULTSFOLDER+"/"+row_conditions[i]+save_filename)
def get_data(base_path,fitfun,descriptors,faults,runs, sens_range, fault_id):
    """
    look up the data in a folder
    :param labels:
    :return:
    """
    individuals={}
    data={}
    behaviour={}
    behaviour_diff={}
    best_data={}

    for descriptor in descriptors:
        best_indexes = []
        condition = descriptor
        data[condition]={}
        behaviour[condition]={}
        behaviour_diff[condition] = {}
        best_data[condition] = {}
        file = open(RESULTSFOLDER+"GEN"+str(FINAL_GEN)+descriptor+"individuals.txt", "w")
        used_indexes=set([])
        for perturbation in faults:
            envir_condition=perturbation
            results=[]
            best_results=[]
            behaviours=[]
            behav_diff=[]
            for run in range(1,runs+1):
                #Outfolder=${ConfigFolder}/results${SUFFIX}; OutputFolder=${Outfolder}${FaultType}${FaultID}
                cfg_folder=base_path+"/"+fitfun+"range"+str(sens_range)+"/"+descriptor
                path=cfg_folder+"/"+perturbation+str(fault_id)+"/"+str(run)+"/fitness"
                temp=read_floats_from_file(open(path,"rb"))
                index=np.argmax(temp)
                used_indexes.add(index)
                M=temp[index]
                results.append(M)
                archive_file = cfg_folder + "/results" + str(run) + "/archive_" + str(FINAL_GEN) + ".dat"
                if run==1:
                    individuals = get_individuals(archive_file, as_string=True)
                    file.write(perturbation+"\t"+str(individuals[index])+"\n")

                print(M)
                print(temp)
                behav=get_bin_performances_uniquearchive(archive_file,as_string=False)
                behav=list(behav.keys())

                if perturbation.endswith("NONE"):
                    # store it to track resilience of a single solution
                    best_indexes.append(index)
                best_index = best_indexes[run-1]
                best_results.append(temp[best_index])
                behav_curr = np.array(behav[index], dtype=float)
                behav_best = np.array(behav[best_index], dtype=float)
                behaviours.append(behav_curr)
                behav_diff.append(abs(behav_curr - behav_best))
            data[condition][envir_condition]=results
            behaviour[condition][envir_condition]=np.std(behaviours)
            best_data[condition][envir_condition]=best_results
            behaviour_diff[condition][envir_condition]=np.mean(behav_diff)
        for i in range(5):
            while True:
                index = np.random.choice(len(individuals))
                if index not in used_indexes:
                    break
            file.write("\t" + str(individuals[index]) + "\n")
        file.close()

    return data,behaviour,best_data, behaviour_diff




if __name__ == "__main__":


    runs =5
    fitfun="Coverage"
    faults=["FAULT_NONE", "FAULT_PROXIMITYSENSORS_SETMIN","FAULT_PROXIMITYSENSORS_SETMAX","FAULT_PROXIMITYSENSORS_SETRANDOM",
            "FAULT_ACTUATOR_LWHEEL_SETHALF", "FAULT_ACTUATOR_RWHEEL_SETHALF", "FAULT_ACTUATOR_BWHEELS_SETHALF"]
    fault_xlabs = ["NONE", "SENSOR_MIN", "SENSOR_MAX","SENSOR_RANDOM","LWHEEL_H","RWHEEL_H", "BWHEELS_H"]
    descriptors=["history","cvt_mutualinfo","cvt_mutualinfoact","cvt_spirit"]

    data_dir =HOME_DIR+"/Data/datanew"
    data,behaviour,best_data, behaviour_diff=get_data(base_path=data_dir,
                  fitfun=fitfun,
                  descriptors=descriptors,
                  faults=faults,
                  runs=5,
                 sens_range=50,
                  fault_id=0)


    make_boxplot(data,row_conditions=descriptors,col_conditions=faults, save_filename=fitfun+"boxplot_faults.png",
                 xlabs=fault_xlabs,ylab="Fitness")

    make_boxplot(best_data, row_conditions=descriptors, col_conditions=faults, save_filename=fitfun + "boxplot_BESTfaults.png",
                 xlabs=fault_xlabs, ylab="Fitness")

    make_barplot(behaviour, row_conditions=descriptors, col_conditions=faults, save_filename=fitfun + "barplot_faults_behavsd.png",
                 xlabs=fault_xlabs,ylab="SD behav")

    make_barplot(behaviour_diff, row_conditions=descriptors, col_conditions=faults, save_filename=fitfun + "barplot_faults_behavdiff.png",
                 xlabs=fault_xlabs,ylab="MAE behav diff")
    print(behaviour)