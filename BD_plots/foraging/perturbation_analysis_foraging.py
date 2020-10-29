
from BD_metrics import *



from plots import *
from foraging.tables import *

import pickle
from foraging.worker_developments import *
from foraging.get_performance import *
from foraging.reference_archive import *
#VE_tags= ["_VE_init"+str(j) for j in [3,4,5,6,8]]
global CENT
global VE_tag
global VE_tags




def bin_single_point(datapoint,minima, bins,bin_sizes):
    category = 0
    cum_prod = 1
    for i in range(len(minima)):  # for each dimension set the binning
        x = minima[i] + bin_sizes[i]
        for j in range(bins):
            if datapoint[i] <= x :
                category+=j*cum_prod
                break
            x += bin_sizes[i]
        cum_prod*=bins

    return category


# def gather_bds(datadir,generation,bd_type,fitfuns,faults,runs,history_type)
#     for bd in bd_type:
#         for fitfun in fitfuns:
#             title = fitfun + "range0.11"
#             prefix = datadir + "/" + title + "/" + bd
#             gather_bds(prefix,generation,bd_type,fitfuns,faults,runs,history_type)


def add_development_of_fault_performance(num_evals,bd_t, r, gener, faultpath,
                          best_performances,time_lost,baseline=False,
                          title_tag="",virtual_energy=False,uniform=False,estimate=True):
    """
    :return:
    """
    virtual_folder="/results" + str(runs[r]) + "/virtual_energy_exp"
    if uniform:
        normal_folder = "/results" + str(runs[r]) + "/uniform"
    else:
        normal_folder ="/results" + str(runs[r])
    path=faultpath+"/fitness" if baseline else faultpath+normal_folder+"/analysis" + str(gener) + "_handcrafted.dat"
    if title_tag.endswith("record"):
        title_tag=title_tag[:-6]
        lines = read_spacedelimited(faultpath + normal_folder + "/" + title_tag + "/BO_output" + VE_tag + "/fitness")
        try:
            fitness=float(lines[-1][0])
        except:
            print(faultpath + normal_folder + "/" + title_tag + "/BO_output" + VE_tag + "/fitness")
            raise Exception()
        return [fitness],  [3600.]
    elif title_tag.startswith("BO"):
        return get_BO_development(bd_t, r, gener, path, faultpath, best_performances,time_lost, normal_folder,virtual_folder,virtual_energy,uniform,estimate)
    elif "single_exp" in title_tag:
        normal_folder+="/"+title_tag
        if title_tag=="single_exp_joint":
            num_workers=1
        else:
            if "agent" in faultpath:
                #get the last
                last = faultpath[-2]
                if last=="p":
                    num_workers=int(faultpath[-1])
                else:
                    num_workers=int(faultpath[-2:])
            elif VE_tag.endswith("2X"):
                num_workers=2*NUM_AGENTS
            elif VE_tag.endswith("4X"):
                num_workers = 4 * NUM_AGENTS
            else:
                num_workers=NUM_AGENTS
        if CENT=="decentralised" and title_tag=="single_exp": # otherwise randomsearch or one GP per robot
            return get_workergroup_developments(num_evals, int(num_workers), path = faultpath + normal_folder + "/BO_output" + VE_tag)
        else:
            return get_worker_developments(num_evals,int(num_workers),faultpath, normal_folder,VE_tag)

        # try:
        #     lines=read_spacedelimited(faultpath+normal_folder+"/"+title_tag+"/BO_output"+VE_tag+"/fitness")
        #     fitness=float(lines[-1][0])
        #     return [fitness],  [3600.]
        # except Exception as e:
        #     print(e)
        #     return [],[]
    else:
        return get_baseline_development(faultpath + normal_folder, title_tag, best_performances,time_lost)




def prepare_data(VE_tags, conditions, settings, max_evals,num_VE_conditions, gener, estimate, by_faulttype):
    # prepare the data for the two conditions
    best_performance_data = []
    time_loss = []
    global CENT
    percentage_eval_data = [[[] for fault in range(num_fault_types[CENT])] for j in range(num_VE_conditions)]
    global VE_tag

    j = 0  # only one fitness function
    for i in range(len(bd_type)):

        for c, condition in enumerate(conditions):
            print(condition)
            title_tag, VE, VE_tag_index = settings[c]
            uniform = condition.endswith("Uniform")
            if "single_exp" in title_tag:
                VE_tag = VE_tag_index # a string
            else:
                if VE_tag_index is not None:
                    VE_tag = VE_tags[VE_tag_index]
                else:
                    VE_tag = None
            print(bd_type[i])
            best_performance_data.append([[[] for t in range(max_evals[c])] for j in range(num_fault_types[CENT])])
            time_loss.append([[[] for t in range(max_evals[c])] for j in range(num_fault_types[CENT])])
            BD_dir = datadir + "/ForagingLarge"
            # get all the data from the archive: no fault

            nofaultpath = BD_dir + "/" + bd_type[i] + "/results"
            nofaultperfs, best_nofaultperfs, maxindsnofault = get_nofault_performances(nofaultpath, gener, runs)

            for fault in foraging_perturbations[CENT]:
                print("fault %s" % (fault))
                for r, run in enumerate(runs):
                    if fault == "software_foodp3f2" and run == 5:
                        print("skipping")
                        continue
                    if fault == "software_foodp4f1" and run == 3:
                        print("skipping")
                        continue
                    faultpath = BD_dir + "/" + bd_type[i] + "/faultyrun" + str(run) + "_" + fault + ""

                    best_performances, time_lost = \
                        add_development_of_fault_performance(max_evals[c],bd_type[i], r, gener, faultpath,
                                                             best_performances=[], time_lost=[], baseline=False,
                                                             title_tag=title_tag, virtual_energy=VE, uniform=uniform,
                                                             estimate=estimate)

                    if by_faulttype:
                        faulttype, index = get_fault_type(fault,CENT)
                        print(index)
                        for t in range(max_evals[c]):

                            best_performance_data[c][index][t] = np.append(best_performance_data[c][index][t],
                                                                           best_performances[t])
                            if VE:
                                time_loss[c][index][t] = np.append(time_loss[c][index][t], time_lost[t]/NUM_TRIALS)
                            else:
                                time_loss[c][index][t] = np.append(time_loss[c][index][t], time_lost[t])
                        if VE:
                            percentage_eval_data[VE_tag_index][index].append(
                                sum([time_lost[i] - time_lost[i - 1] for i in range(1, len(time_lost))]) / (
                                        NUM_SECONDS * max_evals[c]))

                    else:
                        raise Exception("not supported")
        all_data = (best_performance_data, time_loss, percentage_eval_data)
        pickle.dump(all_data, open("../foraging_perturbation_results.pkl", "wb"))
    return all_data




def analyse_development_data(best_performance_data,percentage_eval_data,time_loss,max_evals,conditions, plottag, reference_faultinjection_data, reference_performance_data, settings):
    final_performances=[]

    percentage=[]
    # get the mean and sd for each fault category for this condition (BO or BO-VE)
    bd_index=0
    percentage_file = open("percentage_trial"+plottag+".txt","w")
    table30_file = open("performance_table"+plottag+"30.txt","w")
    # table10_file = open("performance_table" + plottag + "10.txt", "w")
    # table20_file = open("performance_table" + plottag + "20.txt", "w")
    table1_file = open("performance_table" + plottag + "1.txt", "w")

    for fault_category in range(num_fault_types[CENT]):
        print(fault_category)
        percentage.append([])
        performances30 = [None for c in conditions]
        # performances10 =  [None for c in conditions]
        # performances20 =  [None for c in conditions]
        performances1 = [None for c in conditions]
        mean_lines = [[] for c in conditions]
        sd_lines1 = [[] for c in conditions]
        sd_lines2 = [[] for c in conditions]
        reference_category = convert_fault_type(fault_category,CENT)
        min_reference = np.mean(reference_faultinjection_data[bd_index][reference_category])/NUM_AGENTS
        max_reference = np.mean(reference_performance_data[bd_index][reference_category])/NUM_AGENTS
        colors = ["C0","C1","C2", "C3","C4", "C5","C6","C7","C8","C9"]  # colors for the lines
        # (numsides, style, angle)
        markers = ["*", "o","D","X","v","+", "$\dagger$","^","$\spadesuit$","^"]  # markers for the lines
        try:
            percentage_file.write(foraging_fault_types[CENT][fault_category] + " & ")
            table30_file.write(foraging_fault_types[CENT][fault_category] + " & ")
        except:
            print("some error")
        # table10_file.write(foraging_fault_types[fault_category] + " & ")
        # table20_file.write(foraging_fault_types[fault_category] + " & ")
        #table1_file.write(foraging_fault_types[fault_category] + " & ")
        # add the exhaustive search performance as a maximum
        time = [[] for c in conditions]


        for c, condition in enumerate(conditions):
            tag, VE, VE_tag_index = settings[c]
            if VE_tag_index.endswith("2X"):
                scale = 2
            elif VE_tag_index.endswith("4X"):
                scale =4
            else:
                scale = 1
            # if VE_tag_index is not None:
            #     percentage[fault_category].append(np.mean(percentage_eval_data[VE_tag_index][fault_category]))
            #     m_p = np.mean(percentage_eval_data[VE_tag_index][fault_category]) * 100.0
            #     sd_p = np.std(percentage_eval_data[VE_tag_index][fault_category]) * 100.0
            #     percentage_file.write(" & $%.1f$ " % (m_p))
            # after equivalent of 30 evals
            t_3600 = None
            p_3600 = None
            p_sd_3600 = None
            mindist_3600 = float("inf")

            # after equivalent of 10 evals
            # t_1200 = None
            # p_1200 = None
            # p_sd_1200 = None
            # mindist_1200 = float("inf")
            # # after equivalent of 20 evals
            # t_2400 = None
            # p_2400 = None
            # p_sd_2400 = None
            # mindist_2400 = float("inf")
            #
            # # after equivalent of 1 evals
            # t_120 = None
            # p_120 = None
            # p_sd_120 = None
            # mindist_120 = float("inf")
            for t in range(max_evals[c]):
                data = best_performance_data[c][fault_category][t]/NUM_AGENTS
                mean = np.mean(data)
                mean_lines[c].append(mean)
                sd = np.std(data)
                sd_lines1[c].append(mean-sd)
                sd_lines2[c].append(mean+sd)
                consumed = np.mean(time_loss[c][fault_category][t])
                time[c] = np.append(time[c],consumed)
                # if consumed >=0*NUM_SECONDS and consumed <=2*NUM_SECONDS: # try to find closest to 360
                #     dist = abs(consumed - 1*NUM_SECONDS)
                #     if dist < mindist_120:
                #         t_120 = consumed
                #         p_120 = mean
                #         p_sd_120 = sd
                #         mindist_120= dist
                #         performances1[c] = data
                if max_evals[c] == 1 or (consumed >(30*scale - 1)*NUM_SECONDS and consumed <(30*scale + 1)*NUM_SECONDS): # try to find closest to 360
                    dist = abs(consumed - 100*NUM_SECONDS)
                    if dist < mindist_3600:
                        t_3600 = consumed
                        p_3600 = mean
                        p_sd_3600 = sd
                        mindist_3600 = dist
                        performances30[c] = data
                # elif consumed >=9*NUM_SECONDS and consumed <=11*NUM_SECONDS: # try to find closest to 1200
                #     dist = abs(consumed - 10*NUM_SECONDS)
                #     if dist < mindist_1200:
                #         t_1200 = consumed
                #         p_1200 = mean
                #         p_sd_1200 = sd
                #         mindist_1200 = dist
                #         performances10[c] = data
                # elif consumed >=19*NUM_SECONDS and consumed <=21*NUM_SECONDS: # try to find closest to 2400
                #     dist = abs(consumed - 20*NUM_SECONDS)
                #     if dist < mindist_2400:
                #         t_2400 = consumed
                #         p_2400 = mean
                #         p_sd_2400 = sd
                #         mindist_2400 = dist
                #         performances20[c] = data
                elif VE and consumed >= 30*scale*NUM_SECONDS:
                    final_performances=np.append(final_performances,mean - min_reference)
                    break
            # print(str(t_1200) + " " + str(p_1200) + " " + str(p_sd_1200))
            # print(str(t_2400) + " " + str(p_2400) + " " + str(p_sd_2400))
            print(str(t_3600) + " " + str(p_3600) + " " + str(p_sd_3600))
            
            #uncomment for table
            #write_conditional(performances30, c, table30_file, max_reference, min_reference)

        table30_file.write("\n")
        percentage_file.write("\n")

        for c, condition in enumerate(conditions):
            # write_conditional(performances1, c, table1_file, max_reference, min_reference)
            # write_conditional(performances10,c,table10_file,max_reference,min_reference)
            # write_conditional(performances20, c, table20_file,max_reference,min_reference)
            try:
                write_conditional(performances30, c, table30_file,max_reference,min_reference)
            except:
                print("some error")
        # # table10_file.write("$\mathbf{%.2f}$ (+) &" % ((np.mean(performances10[0]) - min_reference)/(max_reference - min_reference)))
        # # table20_file.write("$\mathbf{%.2f}$ (+) &" % ((performances20[0] - min_reference)/(max_reference - min_reference)))
        #table30_file.write("$\mathbf{%.2f}$ (+) &" % ((performances30[0] - min_reference)/(max_reference - min_reference)))
        # table1_file.write("\n")
        # table10_file.write("\n")
        # table20_file.write("\n")
        table30_file.write("\n")
        additional_lines = [(time[0], [min_reference for t in time[0]]), (time[0], [max_reference for t in time[0]])]
        print("index with maximal performance is ",np.argmax(np.mean(performances30,axis=1)))
        if plottag.endswith("record"):
            print("NOT PLOTTING: only one datapoint")
        else:
            createPlot(mean_lines, x_values=time,
                       save_filename="recovery_fault_"+str(foraging_fault_types[CENT][fault_category])+plottag+".pdf", legend_labels=conditions,
                       colors=colors, markers=markers, xlabel="Time ($s$)",
                       ylabel="Best performance",
                       xlim=[0, 4000*scale], xscale="linear", yscale="linear", ylim=[0,4.0],
                       legendbox=(0.10,1.10), annotations=[], xticks=[], yticks=[], task_markers=[], scatter=False,
                       legend_cols=1, legend_fontsize=24, legend_indexes=[], additional_lines=additional_lines, index_x=[],
                       xaxis_style="plain", y_err=[], force=True)#, fill_between=(sd_lines1, sd_lines2))
def development_data(bd_type,runs,gener, by_faulttype=True, max_evals=[30,100],from_file=False, comparison=False, estimate=True):
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

    # get the reference performances
    loadfilename = "data/faulttype/summary_statistics_fault.pkl" # no title tag = exhaustive search
    reference_performance_data,reference_faultinjection_data, _, _, _ = pickle.load(
            open(loadfilename, "rb"))
    global CENT
    if comparison=="centralised":
        conditions = ["SMBO","SMBO-Uniform",
                      "Random", "Gradient-ascent"]
        settings = [("BO", False, None), ("BO",False,None),
                   ("random", False, None), ("gradient_closest", False, None)]
        plottag="LARGE_ALL"
        VE_tags = ["_VE_init" + str(j) for j in [3, 4, 5]]

        CENT="centralised"
        num_VE_conditions=4
    elif comparison=="heterogeneous_params1":
        def hmbo_string(alpha,rho):
            return r"H-SMBO ($alpha="+alpha+r"$,$\rho="+rho+r"$)"
        conditions = [hmbo_string("0.05","0.05"),hmbo_string("0.05","0.1"),hmbo_string("0.05","0.2"),
                      hmbo_string("0.05", "0.4"), hmbo_string("0.05", "1"),
                      hmbo_string("0.25", "0.05"), hmbo_string("0.25", "0.1"), hmbo_string("0.25", "0.2"),
                      hmbo_string("0.25", "0.4"), hmbo_string("0.25", "1")
                      ]
        # settings = [("single_exp", False, "noID"),
        #             ("single_exp_known", False, "final"),
        #             ("single_exp_random", False, "final"),
        #             ("single_exp_randomsearch", False, "final")]
        settings = [
                     ("single_exp", False, "alpha0.05_l0.05_UCB_M52VarNoise"),
                     ("single_exp", False, "alpha0.05_l0.1_UCB_M52VarNoise"),
                     ("single_exp", False, "alpha0.05_l0.2_UCB_M52VarNoise"),
                     ("single_exp", False, "alpha0.05_l0.4_UCB_M52VarNoise"),
                     ("single_exp", False, "alpha0.05_l1_UCB_M52VarNoise"),
                     ("single_exp", False, "alpha0.25_l0.05_UCB_M52VarNoise"),
                     ("single_exp", False, "alpha0.25_l0.1_UCB_M52VarNoise"),
                     ("single_exp", False, "alpha0.25_l0.2_UCB_M52VarNoise"),
                     ("single_exp", False, "alpha0.25_l0.4_UCB_M52VarNoise"),
                     ("single_exp", False, "alpha0.25_l1_UCB_M52VarNoise"),
            ]
        plottag = "HETEROGENEOUS_PARAMS1"
        VE_tags = ["_VE_init" + str(j) for j in range(10)]
        num_VE_conditions = 10
    elif comparison == "heterogeneous_params2":
        def hmbo_string(alpha, rho):
            return r"H-SMBO ($alpha=" + alpha + r"$,$\rho=" + rho + r"$)"

        conditions = [
                      hmbo_string("0.05", "0.05"), hmbo_string("0.05", "0.1"), hmbo_string("0.05", "0.2"),
                      hmbo_string("0.05", "0.4"), hmbo_string("0.05", "1"),
                      hmbo_string("0.25", "0.05"), hmbo_string("0.25", "0.1"), hmbo_string("0.25", "0.2"),
                      hmbo_string("0.25", "0.4"), hmbo_string("0.25", "1"),
                      hmbo_string("0.50", "0.05"), hmbo_string("0.50", "0.1"), hmbo_string("0.50", "0.2"),
                      hmbo_string("0.50", "0.4"), hmbo_string("0.50", "1"),
                      hmbo_string("1", "0.05"), hmbo_string("1", "0.1"), hmbo_string("1", "0.2"),
                      hmbo_string("1", "0.4"), hmbo_string("1", "1")
                      ]
        # settings = [("single_exp", False, "noID"),
        #             ("single_exp_known", False, "final"),
        #             ("single_exp_random", False, "final"),
        #             ("single_exp_randomsearch", False, "final")]
        settings = [
            ("single_exp", False, "alpha0.50_l0.05_UCB_M52VarNoise"),
            ("single_exp", False, "alpha0.50_l0.1_UCB_M52VarNoise"),
            ("single_exp", False, "alpha0.50_l0.2_UCB_M52VarNoise"),
            ("single_exp", False, "alpha0.50_l0.4_UCB_M52VarNoise"),
            ("single_exp", False, "alpha0.50_l1_UCB_M52VarNoise"),
            ("single_exp", False, "alpha1_l0.05_UCB_M52VarNoise"),
            ("single_exp", False, "alpha1_l0.1_UCB_M52VarNoise"),
            ("single_exp", False, "alpha1_l0.2_UCB_M52VarNoise"),
            ("single_exp", False, "alpha1_l0.4_UCB_M52VarNoise"),
            ("single_exp", False, "alpha1_l1_UCB_M52VarNoise"),
        ]
        plottag = "HETEROGENEOUS_PARAMS2"
        VE_tags = ["_VE_init" + str(j) for j in range(10)]
        num_VE_conditions = 10
    elif comparison=="heterogeneous_params":

        def hmbo_string(alpha, rho):
            return r"H-SMBO ($alpha=" + alpha + r"$,$\rho=" + rho + r"$)"

        conditions = [
            hmbo_string("0.05", "0.05"), hmbo_string("0.05", "0.1"), hmbo_string("0.05", "0.2"),
            hmbo_string("0.05", "0.4"), hmbo_string("0.05", "1"),
            hmbo_string("0.25", "0.05"), hmbo_string("0.25", "0.1"), hmbo_string("0.25", "0.2"),
            hmbo_string("0.25", "0.4"), hmbo_string("0.25", "1"),
                    hmbo_string("0.50", "0.05"), hmbo_string("0.50", "0.1"), hmbo_string("0.50", "0.2"),
                      hmbo_string("0.50", "0.4"), hmbo_string("0.50", "1"),
                      hmbo_string("1", "0.05"), hmbo_string("1", "0.1"), hmbo_string("1", "0.2"),
                      hmbo_string("1", "0.4"), hmbo_string("1", "1")
                      ]
        # settings = [("single_exp", False, "noID"),
        #             ("single_exp_known", False, "final"),
        #             ("single_exp_random", False, "final"),
        #             ("single_exp_randomsearch", False, "final")]
        settings = [
            ("single_exp", False, "alpha0.05_l0.05_UCB_M52VarNoise"),
            ("single_exp", False, "alpha0.05_l0.1_UCB_M52VarNoise"),
            ("single_exp", False, "alpha0.05_l0.2_UCB_M52VarNoise"),
            ("single_exp", False, "alpha0.05_l0.4_UCB_M52VarNoise"),
            ("single_exp", False, "alpha0.05_l1_UCB_M52VarNoise"),
            ("single_exp", False, "alpha0.25_l0.05_UCB_M52VarNoise"),
            ("single_exp", False, "alpha0.25_l0.1_UCB_M52VarNoise"),
            ("single_exp", False, "alpha0.25_l0.2_UCB_M52VarNoise"),
            ("single_exp", False, "alpha0.25_l0.4_UCB_M52VarNoise"),
            ("single_exp", False, "alpha0.25_l1_UCB_M52VarNoise"),
            ("single_exp", False, "alpha0.50_l0.05_UCB_M52VarNoise"),
            ("single_exp", False, "alpha0.50_l0.1_UCB_M52VarNoise"),
            ("single_exp", False, "alpha0.50_l0.2_UCB_M52VarNoise"),
            ("single_exp", False, "alpha0.50_l0.4_UCB_M52VarNoise"),
            ("single_exp", False, "alpha0.50_l1_UCB_M52VarNoise"),
            ("single_exp", False, "alpha1_l0.05_UCB_M52VarNoise"),
            ("single_exp", False, "alpha1_l0.1_UCB_M52VarNoise"),
            ("single_exp", False, "alpha1_l0.2_UCB_M52VarNoise"),
            ("single_exp", False, "alpha1_l0.4_UCB_M52VarNoise"),
            ("single_exp", False, "alpha1_l1_UCB_M52VarNoise"),
        ]
        plottag = "HETEROGENEOUS_PARAMS2"
        VE_tags = ["_VE_init" + str(j) for j in range(20)]
        num_VE_conditions = 20
    elif comparison=="decentralised":
        conditions = ["SMBO-Dec Local","SMBO-Dec Naive","SMBO No Sharing","Random No sharing"]
        # settings = [("single_exp", False, "noID"),
        #             ("single_exp_known", False, "final"),
        #             ("single_exp_random", False, "final"),
        #             ("single_exp_randomsearch", False, "final")]
        settings = [
                    ("single_exp", False, "alpha0.93_l0.12_UCB_LOCAL_M52VarNoise"),
                    ("single_exp", False, "alpha0.93_l0.12_UCB_M52VarNoise"),
                    ("single_exp_independent", False, "alpha0.93_l0.12_UCB_M52VarNoise"),
                    ("single_exp_randomsearch",False, "alpha0.93_l0.12_UCB_M52VarNoise")
        ]
        plottag="LARGE_DECENTRALISED"
        VE_tags = ["_VE_init" + str(j) for j in [3, 4,5, 6]]
        CENT = "decentralised"
        num_VE_conditions=4
    elif comparison=="decentralised2X":
        conditions = ["SMBO-Dec Local","SMBO-Dec Naive","SMBO No Sharing","Random No sharing"]
        # settings = [("single_exp", False, "noID"),
        #             ("single_exp_known", False, "final"),
        #             ("single_exp_random", False, "final"),
        #             ("single_exp_randomsearch", False, "final")]
        settings = [
                    ("single_exp", False, "alpha0.93_l0.12_UCB_LOCAL_M52VarNoise_2X"),
                    ("single_exp", False, "alpha0.93_l0.12_UCB_M52VarNoise_2X"),
                    ("single_exp_independent", False, "alpha0.93_l0.12_UCB_M52VarNoise_2X"),
                   ("single_exp_randomsearch",False, "alpha0.93_l0.12_UCB_M52VarNoise_2X")
        ]
        plottag="LARGE_DECENTRALISED2X"
        VE_tags = ["_VE_init" + str(j) for j in [3, 4,5, 6]]
        CENT = "decentralised"
        num_VE_conditions=4
    elif comparison=="decentralised4X":
        conditions = ["SMBO-Dec Local","SMBO-Dec Naive","SMBO No Sharing","Random No sharing"]
        # settings = [("single_exp", False, "noID"),
        #             ("single_exp_known", False, "final"),
        #             ("single_exp_random", False, "final"),
        #             ("single_exp_randomsearch", False, "final")]
        settings = [
                    ("single_exp", False, "alpha0.93_l0.12_UCB_LOCAL_M52VarNoise_4X"),
                    ("single_exp", False, "alpha0.93_l0.12_UCB_M52VarNoise_4X"),
                    ("single_exp_independent", False, "alpha0.93_l0.12_UCB_M52VarNoise_4X"),
                   ("single_exp_randomsearch",False, "alpha0.93_l0.12_UCB_M52VarNoise_4X")
        ]
        plottag="LARGE_DECENTRALISED4X"
        VE_tags = ["_VE_init" + str(j) for j in [3, 4,5, 6]]
        CENT = "decentralised"
        num_VE_conditions=4
    elif comparison=="decentralised_record":
        conditions = ["SMBO-Dec Local","SMBO-Dec Naive","SMBO No Sharing","Random No sharing"]
        # settings = [("single_exp", False, "noID"),
        #             ("single_exp_known", False, "final"),
        #             ("single_exp_random", False, "final"),
        #             ("single_exp_randomsearch", False, "final")]
        settings = [
                    ("single_exprecord", False, "alpha0.93_l0.12_UCB_LOCAL_M52VarNoise"),
                    ("single_exprecord", False, "alpha0.93_l0.12_UCB_M52VarNoise"),
                    ("single_exp_independentrecord", False, "alpha0.93_l0.12_UCB_M52VarNoise"),
                    ("single_exp_randomsearchrecord",False, "alpha0.93_l0.12_UCB_M52VarNoise")
        ]
        plottag="record"
        VE_tags = ["_VE_init" + str(j) for j in [3, 4,5, 6]]
        CENT = "decentralised"
        num_VE_conditions=4
    elif comparison=="decentralised_record2X":
        conditions = ["SMBO-Dec Local","SMBO-Dec Naive","SMBO No Sharing","Random No sharing"]
        # settings = [("single_exp", False, "noID"),
        #             ("single_exp_known", False, "final"),
        #             ("single_exp_random", False, "final"),
        #             ("single_exp_randomsearch", False, "final")]
        settings = [
                    ("single_exprecord", False, "alpha0.93_l0.12_UCB_LOCAL_M52VarNoise_2X"),
                    ("single_exprecord", False, "alpha0.93_l0.12_UCB_M52VarNoise_2X"),
                    ("single_exp_independentrecord", False, "alpha0.93_l0.12_UCB_M52VarNoise_2X"),
                    ("single_exp_randomsearchrecord",False, "alpha0.93_l0.12_UCB_M52VarNoise_2X")
        ]
        plottag="record"
        VE_tags = ["_VE_init" + str(j) for j in [3, 4,5, 6]]
        CENT = "decentralised"
        num_VE_conditions=4
    elif comparison=="fest":
        conditions = ["SMBO", "VE-SMBO E(0)=3","VE-SMBO E(0)=4","VE-SMBO E(0)=5","VE-SMBO E(0)=6","VE-SMBO E(0)=8"]
        settings = [("BO", False, None), ("BO", True, 0), ("BO", True, 1), ("BO", True, 2), ("BO", True, 3),("BO", True, 4)]
        plottag="fest_params"
        VE_tags = ["_nocollision_init" + str(j) for j in [3, 4, 5, 6, 8]]
        num_VE_conditions = 5
    elif comparison=="VE":
        conditions = ["SMBO", "VE-SMBO E(0)=5", "VE-SMBO E(0)=10"]
        settings = [("BO", False, None), ("BO", True, 0),("BO", True, 1)]
        plottag="VE_params"
        num_VE_conditions = 5
        VE_tags = ["_init" + str(j) + "_single" for j in [5,10]]
    else:
        raise Exception()

    if estimate:
        plottag+="estimate"
    if from_file:
        best_performance_data, time_loss, percentage_eval_data = pickle.load(open(
            "../foraging_perturbation_results.pkl", "rb"))
    else:
        best_performance_data, time_loss, percentage_eval_data = prepare_data(VE_tags,conditions, settings, max_evals,num_VE_conditions, gener, estimate,by_faulttype)

    analyse_development_data(best_performance_data,percentage_eval_data,time_loss,max_evals,conditions, plottag, reference_faultinjection_data, reference_performance_data, settings)


    #r = np.corrcoef(percentage, final_performances)
    #print("correlation="+str(r))



# def test_significance(bd_type,by_faulttype,data_type):
#
#
#     best_performance_data, best_transfer_data, trial_data, performance_loss_data, num_trials = pickle.load(open("data/faulttype/summary_statistics_fault.pkl", "rb"))
#     best_BOperformance_data, best_BOtransfer_data, trial_BOdata, performance_BOloss_data, num_BOtrials = pickle.load(open("data/faulttype/summary_statistics_faultBO.pkl", "rb"))
#     best_BOVEperformance_data, best_BOVWtransfer_data, trial_BOVEdata, performance_BOVEloss_data, num_BOVEtrials = pickle.load(open("data/faulttype/summary_statistics_faultBOVE.pkl", "rb"))
#
#     for f in range(num_fault_types[CENT]):
#         print(foraging_fault_types[f])
#         for i in range(len(bd_type)):
#                 print("compare exhaustive before and after")
#                 x = best_performance_data[i][f]
#                 y = best_transfer_data[i][f]
#                 stat,p = ranksums(x,y)
#                 print("%s after vs %s before: U=%.2f, p=%.6f"%(bd_type[i],bd_type[i],stat,p))
#                 delta, label= cliffs_delta(stat,x,y)
#                 print("Cliffs delta: %.3f   %s"%(delta,label))
#
#                 print("-------------------------------------")
#
#                 print("compare exhaustive to BO")
#                 y = best_BOperformance_data[i][f]
#                 x = best_performance_data[i][f]
#                 stat, p = ranksums(x, y)
#                 print("%s exhaustive vs %s BO: U=%.2f, p=%.6f" % (bd_type[i], bd_type[i], stat, p))
#                 delta, label = cliffs_delta(stat, x, y)
#                 print("Cliffs delta: %.3f   %s" % (delta, label))
#
#
#                 print("--------------------------------------")
#                 print("compare BO  virtual vs BO")
#                 y = best_BOperformance_data[i][f]
#                 x = best_BOVEperformance_data[i][f]
#                 stat, p = ranksums(x, y)
#                 print("%s virtual_energy vs %s BO: U=%.2f, p=%.6f" % (bd_type[i], bd_type[i], stat, p))
#                 delta, label = cliffs_delta(stat, x, y)
#                 print("Cliffs delta: %.3f   %s" % (delta, label))
#
#
#                 print("")


def test_significance(bd_type,by_faulttype,data_type, max_evals):
    conditions = ["SMBO", "VE-SMBO E(0)=3", "VE-SMBO E(0)=4", "VE-SMBO E(0)=5", "VE-SMBO E(0)=6", "VE-SMBO E(0)=8",
                  "Random", "Gradient-ascent"]
    settings = [("BO", False, None), ("BO", True, 0), ("BO", True, 1), ("BO", True, 2), ("BO", True, 3),
                ("BO", True, 4), ("random", False, None), ("gradient_closest", False, None)]
    num_VE_conditions = 5
    best_performance_data, time_loss, percentage_eval_data = pickle.load(open("../foraging_perturbation_results.pkl", "rb"))
    percentage = []
    global CENT
    for fault_category in range(num_fault_types[CENT]):

        for c, condition in enumerate(conditions):
            tag, VE, VE_tag_index = settings[c]
            for t in range(max_evals[c]):
                if np.mean(time_loss[c][fault_category][t]) >= 3600.0:
                    p = best_performance_data[c][fault_category][t]
                    final_performances = np.append(final_performances, p)
                    break




def plot_histogram(bd_type,by_fitfun=True):
    if not by_fitfun:
        best_performance_data, performance_data, transfer_data, resilience_data = pickle.load(
            open("summary_statistics.pkl", "rb"))
    else:
        best_performance_data, performance_data, transfer_data, resilience_data = pickle.load(
            open("summary_statistics_fitfun.pkl", "rb"))
    num_bins = 10
    for f in range(len(fitfuns)):
        print(fitfuns[f])
        for i in range(len(bd_type)):
            x = resilience_data[i][f]
            plt.figure()
            n, bins, patches = plt.hist(x, num_bins, facecolor='blue', alpha=0.5,range=(-0.30, 0))
            plt.savefig("HIST"+bd_type[i]+fitfuns[f]+".pdf")


def determine_noise():
    BD_dir = datadir + "/ForagingLarge"
    # get all the data from the archive: no fault

    nofaultpath = BD_dir + "/history/results"
    nofaultperfs, best_nofaultperfs, maxindsnofault = get_nofault_performances(nofaultpath, "20000", runs)

    v = [np.var(p) for p in nofaultperfs]
    print(v)




    #

if __name__ == "__main__":

    #development_data(bd_type, runs, 20000, by_faulttype=True, max_evals=[30,30,30,30],from_file=False,comparison="decentralised",estimate=False)
    development_data(bd_type, runs, 20000, by_faulttype=True, max_evals=[30,30,30,30],from_file=False,comparison="decentralised2X",estimate=False)

    development_data(bd_type, runs, 20000, by_faulttype=True, max_evals=[1, 1, 1, 1], from_file=False,
                      comparison="decentralised_record2X", estimate=False)


    #development_data(bd_type, runs, 20000, by_faulttype=True, max_evals=[20,20,20,20],from_file=False,comparison="decentralised4X",estimate=False)

