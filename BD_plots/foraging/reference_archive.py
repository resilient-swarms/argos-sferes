
from foraging.foraging_params import *
from foraging.tables import *
from foraging.get_performance import *



def add_fault_performance(bd_t, r, gener, nofaultperfs,best_nofaultperfs,maxindsnofault,faultpath,
                          best_performances,best_transfer,resilience, baseline=False,
                          title_tag="",virtual_energy=False, VE_tag=""):
    """

    :param j: fitfun index
    :param r: run index
    :param best_nofaultperfs:
    :param maxindsnofault:
    :param faultpath:
    :param best_performances:
    :param performances:
    :param best_transfer:
    :param transfer:
    :param recovery:
    :param resilience:
    :param baseline:
    :return:
    """
    virtual_folder="/results" + str(runs[r]) + "/virtual_energy_exp"
    exhaustive_virtual_folder="/results" + str(runs[r]) + "/virtual_energy_exp/exhaustive"
    normal_folder ="/results" + str(runs[r])
    path=faultpath+"/fitness" if baseline else faultpath+normal_folder+"/analysis" + str(gener) + "_handcrafted.dat"
    virtualpath = faultpath + virtual_folder + "/fitness"
    if baseline:

        best_performance=get_baseline_fitness(path)
    else:

        temp = np.array(list(get_ind_performances_uniquearchive(path).values())).flatten()
        #performances = np.append(performances, temp)

        if title_tag.startswith("BO"):

            if virtual_energy:
                BOfile = faultpath + virtual_folder +"/BO_output" + VE_tag + "/best_observations.dat"
            else:
                BOfile = faultpath + normal_folder + "/BO_output" + settings_tag + "/best_observations.dat"
            parsed_file_list = read_spacedelimited(BOfile)

            if virtual_energy:
                best_performance,_=get_best_performance_VE(path,faultpath,virtual_folder,bd_t,r,gener, VE_tag)
            else:
                last_line = parsed_file_list[-1]
                best_performance=float(last_line[-1])
            num_trials=len(parsed_file_list) - 1 # count the lines, but top line does not count

            if virtual_energy:
                BOfile = faultpath + virtual_folder + "/BO_output" + VE_tag + "/observations.dat"
            else:
                BOfile = faultpath + normal_folder + "/BO_output" + settings_tag + "/observations.dat"
            parsed_file_list = read_spacedelimited(BOfile)
            performance_loss=0.
            i=0
            time_loss=0.
            for line in parsed_file_list:
                if i == 0:
                    i += 1
                    continue
                if virtual_energy:
                    # time_consumed = min(float(line[-1]), TICKS_PER_TRIAL)/TICKS_PER_SECOND
                    performance, time_consumed = index2fullperformance(bd_t,path,faultpath,virtual_folder,i,r,gener, VE_tag)

                    time_loss += time_consumed
                    performance_loss += (best_performance - performance)*(time_consumed/NUM_SECONDS) # multiply by trial proportion
                else:
                    performance_loss+=(best_performance - float(line[-1])) # all have equal time

                i+=1
            if not virtual_energy:
                time_loss = num_trials * NUM_SECONDS

        else:
            maxind, best_performance = get_best_individual(path, add_performance=True, index_based=True)
            time_loss=None
            performance_loss=None
            num_trials=None

        # all performances vs all nofaultperformances
        # for k in range(len(nofaultperfs[r])):
        #     transfer = np.append(transfer, [(temp[k] - nofaultperfs[r][k]) / baseline_performances[fitfuns[j]]])  # avoid NANs

        best_transfer = np.append(best_transfer,temp[maxindsnofault[r]])

    best_performances = np.append(best_performances, best_performance)

    #recovery = np.append(recovery, [(best_performance - best_nofaultperfs[r]) / baseline_performances[
    #    fitfuns[j]]])  # best performance vs best nofaultperf
    #resilience = np.append(resilience, (best_performance - best_nofaultperfs[r]) / best_nofaultperfs[r])

    return best_performances,  best_transfer, time_loss, performance_loss, num_trials




def reference_archive(fitfuns,fitfunlabels,bd_type,runs,gener, by_faulttype=True, load_existing=False,title_tag="",virtual_energy=False):
    """

    performance: defined as the performance on all the perturbed environments
    transfer: defined as each individuals' drop in performance
    resilience: the best performance's drop in performance



    :param fitfuns:
    :param bd_type:
    :param runs:
    :param faults:
    :param gener:
    :return:
    """
    global CENT
    CENT="centralised"
    if virtual_energy:
        title_tag += "VE"

    loadfilename = "../data/faulttype/summary_statistics_fault"+title_tag+".pkl" if by_faulttype else "data/combined/summary_statistics"+title_tag+".pkl"
    if load_existing:
        best_performance_data, performance_data, best_transfer_data, transfer_data, resilience_data = pickle.load(
            open(loadfilename, "rb"))
        # baselinefilename = "data/fitfun/summary_statistics_fitfunbaseline.pkl"
        # bp_data, p_data, bt_data, t_data, r_data, re_data = pickle.load(open(baselinefilename, "rb"))
        # if "baseline" in bd_type:
        #     best_performance_data.append(bp_data[0])
        #     performance_data.append(p_data[0])
        #     best_transfer_data.append(bt_data[0])
        #
        #     transfer_data.append(t_data[0])
        #     #recovery_data.append(r_data[0])
        #     resilience_data.append(re_data[0])
        # pickle.dump((best_performance_data, performance_data, best_transfer_data, transfer_data, resilience_data),open("data/fitfun/summary_statistics_fitfun.pkl", "wb"))

    else:
        best_performance_data = []
        performance_data = []
        transfer_data = []
        best_transfer_data = []
        resilience_data = []
        recovery_data = []
        trial_data = []
        num_trials = []
        performance_loss_data=[]
        j = 0  #only one fitness function
        for i in range(len(bd_type)):
            print(bd_type[i])
            best_performance_data.append([[] for j in range(num_fault_types[CENT])])
            performance_data.append([[] for j in range(num_fault_types[CENT])])
            best_transfer_data.append([[] for j in range(num_fault_types[CENT])])
            transfer_data.append([[] for j in range(num_fault_types[CENT])])
            resilience_data.append([[] for j in range(num_fault_types[CENT])])
            recovery_data.append([[] for j in range(num_fault_types[CENT])])
            trial_data.append([[] for j in range(num_fault_types[CENT])])
            num_trials.append([[] for j in range(num_fault_types[CENT])])
            performance_loss_data.append([[] for j in range(num_fault_types[CENT])])

            BD_dir = datadir+"/ForagingLarge"
                # get all the data from the archive: no fault

            nofaultpath=BD_dir + "/" + bd_type[i] + "/results"
            nofaultperfs, best_nofaultperfs, maxindsnofault = get_nofault_performances(nofaultpath,gener,runs)

            perturbs = foraging_perturbations[CENT]
            for fault in perturbs:
                print("fault %s"%(fault))
                for r, run in enumerate(runs):
                    if fault=="software_foodp3f2" and run==5:
                        print("skipping")
                        continue
                    if fault=="software_foodp4f1" and run==3:
                        print("skipping")
                        continue

                    faultpath = BD_dir + "/" + bd_type[i] + "/faultyrun" + str(run) + "_" + fault + ""

                    best_performances, best_transfer,  _trial_time, _performance_loss, _num_trials = add_fault_performance(bd_type[i], r, gener, nofaultperfs, best_nofaultperfs, maxindsnofault, faultpath,
                                                                                                                          best_performances=[], best_transfer=[],resilience=[], baseline=bd_type[i]=="baseline",
                                                                                                              title_tag=title_tag,virtual_energy=virtual_energy,VE_tag=VE_tag)



                    #otherwise transfer is undefined; we observe f=0 for some individuals in bordercoverage
                    # print(transfer.max())
                    # print(np.mean(transfer))


                    if by_faulttype:
                        faulttype,index=get_fault_type(fault,CENT)
                        best_performance_data[i][index] = np.append(best_performance_data[i][index],best_performances)
                        best_transfer_data[i][index] = np.append(best_transfer_data[i][index],best_transfer)
                        if title_tag.startswith("BO"):
                            trial_data[i][index].append(_trial_time)
                            num_trials[i][index].append(_num_trials)
                            #performance_loss_data[i][index].append(_performance_loss / 6.0)
                    else:
                        # normalise by maximal fitness (make different performance data comparable)
                        best_performance_data[i] = np.append(best_performance_data[i],best_performances/baseline_performances[fitfuns[j]])
                        best_transfer_data[i] = np.append(best_transfer_data[i],best_transfer)
                        #resilience_data[i] = np.append(resilience_data[i],resilience)
                        trial_data[i].append(_trial_time)
                        num_trials[i].append(_num_trials)

    all_data = (best_performance_data, best_transfer_data, trial_data, performance_loss_data, num_trials)

    if by_faulttype:
        if not load_existing:
            pickle.dump(all_data,
                        open(loadfilename, "wb"))

        # for i in range(len(best_performance_data)):
        #     for j in range(len(fitfuns)):
        #         best_performance_data[i][j]/=baseline_performances[fitfuns[j]]
        if title_tag.startswith("BO"):
            part_data = (best_performance_data, trial_data, num_trials)
            col_labels =  [("Recovery performance", "float2"),("Evaluation time ($s$)", "float2"), ("Number of evaluations","float2") ]
        else:
            part_data = (best_transfer_data,best_performance_data)
            col_labels=[("Fault injection performance","float2"),("Recovery performance","float2")]


        with open("../results/fault/summary_table_faulttype"+title_tag,"w") as f:
            make_table(f,part_data,
                       rowlabels=foraging_fault_types[CENT],
                       columnlabels=legend_labels,
                       conditionalcolumnlabels=col_labels,
                       transpose=False)
        with open("../results/fault/summary_table_faulttype_median"+title_tag,"w") as f:
            make_table(f,part_data,
                       rowlabels=foraging_fault_types[CENT],
                       columnlabels=legend_labels,
                       conditionalcolumnlabels=col_labels,
                       median=True,
                       transpose=False)
    else:
        if not load_existing:
            pickle.dump(all_data,
                        open(loadfilename, "wb"))
        part_data = [best_transfer_data, best_performance_data, resilience_data]
        with open("../results/summary_table"+title_tag,"w") as f:
            make_table(f,part_data,
                       rowlabels=legend_labels,
                       columnlabels=[],
                       conditionalcolumnlabels=[("besttransfer","float2"),("bestperformance","float2"),("resilience","float2")])
        with open("../results/summary_table_median"+title_tag, "w") as f:
            make_table(f, part_data,
                           rowlabels=legend_labels,
                           columnlabels=[],
                           conditionalcolumnlabels=[("besttransfer","float2"),("bestperformance","float2"),("resilience","float2")],
                            median=True)
def reference_archive(fitfuns,fitfunlabels,bd_type,runs,gener, by_faulttype=True, load_existing=False,title_tag="",virtual_energy=False):
    """

    performance: defined as the performance on all the perturbed environments
    transfer: defined as each individuals' drop in performance
    resilience: the best performance's drop in performance



    :param fitfuns:
    :param bd_type:
    :param runs:
    :param faults:
    :param gener:
    :return:
    """
    CENT="centralised"
    if virtual_energy:
        title_tag += "VE"

    loadfilename = HOME_DIR + "/argos-sferes/BD_plots/data/faulttype/summary_statistics_fault"+title_tag+".pkl" if by_faulttype else "data/combined/summary_statistics"+title_tag+".pkl"
    if load_existing:
        best_performance_data, performance_data, best_transfer_data, transfer_data, resilience_data = pickle.load(
            open(loadfilename, "rb"))
        # baselinefilename = "data/fitfun/summary_statistics_fitfunbaseline.pkl"
        # bp_data, p_data, bt_data, t_data, r_data, re_data = pickle.load(open(baselinefilename, "rb"))
        # if "baseline" in bd_type:
        #     best_performance_data.append(bp_data[0])
        #     performance_data.append(p_data[0])
        #     best_transfer_data.append(bt_data[0])
        #
        #     transfer_data.append(t_data[0])
        #     #recovery_data.append(r_data[0])
        #     resilience_data.append(re_data[0])
        # pickle.dump((best_performance_data, performance_data, best_transfer_data, transfer_data, resilience_data),open("data/fitfun/summary_statistics_fitfun.pkl", "wb"))

    else:
        best_performance_data = []
        performance_data = []
        transfer_data = []
        best_transfer_data = []
        resilience_data = []
        recovery_data = []
        trial_data = []
        num_trials = []
        performance_loss_data=[]
        j = 0  #only one fitness function
        for i in range(len(bd_type)):
            print(bd_type[i])
            best_performance_data.append([[] for j in range(num_fault_types[CENT])])
            performance_data.append([[] for j in range(num_fault_types[CENT])])
            best_transfer_data.append([[] for j in range(num_fault_types[CENT])])
            transfer_data.append([[] for j in range(num_fault_types[CENT])])
            resilience_data.append([[] for j in range(num_fault_types[CENT])])
            recovery_data.append([[] for j in range(num_fault_types[CENT])])
            trial_data.append([[] for j in range(num_fault_types[CENT])])
            num_trials.append([[] for j in range(num_fault_types[CENT])])
            performance_loss_data.append([[] for j in range(num_fault_types[CENT])])

            BD_dir = datadir+"/ForagingLarge"
                # get all the data from the archive: no fault

            nofaultpath=BD_dir + "/" + bd_type[i] + "/results"
            nofaultperfs, best_nofaultperfs, maxindsnofault = get_nofault_performances(nofaultpath,gener,runs)

            perturbs = foraging_perturbations[CENT]
            for fault in perturbs:
                print("fault %s"%(fault))
                for r, run in enumerate(runs):
                    if fault=="software_foodp3f2" and run==5:
                        print("skipping")
                        continue
                    if fault=="software_foodp4f1" and run==3:
                        print("skipping")
                        continue

                    faultpath = BD_dir + "/" + bd_type[i] + "/faultyrun" + str(run) + "_" + fault + ""

                    best_performances, best_transfer,  _trial_time, _performance_loss, _num_trials = add_fault_performance(bd_type[i], r, gener, nofaultperfs, best_nofaultperfs, maxindsnofault, faultpath,
                                                                                                                          best_performances=[], best_transfer=[],resilience=[], baseline=bd_type[i]=="baseline",
                                                                                                              title_tag=title_tag,virtual_energy=virtual_energy,VE_tag="")



                    #otherwise transfer is undefined; we observe f=0 for some individuals in bordercoverage
                    # print(transfer.max())
                    # print(np.mean(transfer))


                    if by_faulttype:
                        faulttype,index=get_fault_type(fault,CENT)
                        best_performance_data[i][index] = np.append(best_performance_data[i][index],best_performances)
                        best_transfer_data[i][index] = np.append(best_transfer_data[i][index],best_transfer)
                        if title_tag.startswith("BO"):
                            trial_data[i][index].append(_trial_time)
                            num_trials[i][index].append(_num_trials)
                            #performance_loss_data[i][index].append(_performance_loss / 6.0)
                    else:
                        # normalise by maximal fitness (make different performance data comparable)
                        best_performance_data[i] = np.append(best_performance_data[i],best_performances/baseline_performances[fitfuns[j]])
                        best_transfer_data[i] = np.append(best_transfer_data[i],best_transfer)
                        #resilience_data[i] = np.append(resilience_data[i],resilience)
                        trial_data[i].append(_trial_time)
                        num_trials[i].append(_num_trials)

    all_data = (best_performance_data, best_transfer_data, trial_data, performance_loss_data, num_trials)

    if by_faulttype:
        if not load_existing:
            pickle.dump(all_data,
                        open(loadfilename, "wb"))

        # for i in range(len(best_performance_data)):
        #     for j in range(len(fitfuns)):
        #         best_performance_data[i][j]/=baseline_performances[fitfuns[j]]
        if title_tag.startswith("BO"):
            part_data = (best_performance_data, trial_data, num_trials)
            col_labels =  [("Recovery performance", "float2"),("Evaluation time ($s$)", "float2"), ("Number of evaluations","float2") ]
        else:
            part_data = (best_transfer_data,best_performance_data)
            col_labels=[("Fault injection performance","float2"),("Recovery performance","float2")]


        with open("../results/fault/summary_table_faulttype"+title_tag,"w") as f:
            make_table(f,part_data,
                       rowlabels=foraging_fault_types[CENT],
                       columnlabels=legend_labels,
                       conditionalcolumnlabels=col_labels,
                       transpose=False)
        with open("../results/fault/summary_table_faulttype_median"+title_tag,"w") as f:
            make_table(f,part_data,
                       rowlabels=foraging_fault_types[CENT],
                       columnlabels=legend_labels,
                       conditionalcolumnlabels=col_labels,
                       median=True,
                       transpose=False)
    else:
        if not load_existing:
            pickle.dump(all_data,
                        open(loadfilename, "wb"))
        part_data = [best_transfer_data, best_performance_data, resilience_data]
        with open("../results/summary_table"+title_tag,"w") as f:
            make_table(f,part_data,
                       rowlabels=legend_labels,
                       columnlabels=[],
                       conditionalcolumnlabels=[("besttransfer","float2"),("bestperformance","float2"),("resilience","float2")])
        with open("../results/summary_table_median"+title_tag, "w") as f:
            make_table(f, part_data,
                           rowlabels=legend_labels,
                           columnlabels=[],
                           conditionalcolumnlabels=[("besttransfer","float2"),("bestperformance","float2"),("resilience","float2")],
                            median=True)


if __name__ == "main":
    reference_archive(fitfuns, fitfunlabels, bd_type, runs, generation, by_faulttype=True, load_existing=False,
                      title_tag="", virtual_energy=False)