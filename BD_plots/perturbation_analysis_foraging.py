from foraging_params import *
from BD_metrics import *
from reduce_translated_archive import *

RESULTSFOLDER="results"

from plots import *

from significance import *
import pickle

NUM_AGENTS=6.0

settings_tag="_100evaluations"
uniform_tag=""
#VE_tags= ["_VE_init"+str(j) for j in [3,4,5,6,8]]
global VE_tags
global VE_tag
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


def index2fullperformance(bd_t,path,faultpath, virtual_folder, best_index, r, gener, tag, estimate):
    # get the corresponding BD
    samp = faultpath + virtual_folder + "/BO_output" + tag + "/samples.dat"
    samp_list = read_spacedelimited(samp)
    sample = None
    i = 0
    for line in samp_list:
        if i == best_index:
            sample = tuple(line[1:])
            break
        i += 1
    # get the corresponding individual in the normal environment, original archive
    original_archive = HOME_DIR + "/Data/Foraging/"+bd_t+ "/results" + str(runs[r]) + "/archive_" + str(gener) + ".dat"
    archive = read_spacedelimited(original_archive)
    archive_individual = None
    for item in archive:
        bd = tuple(item[1:-1])
        if bd == sample:
            archive_individual = item[0]
            break
    if archive_individual is None:
        raise Exception("sample not found in normal archive")


    # if performance is None:
    #     raise Exception("individual not found in faulty archive")
    timefile = faultpath + virtual_folder + "/BO_output" + tag + "/fitness" + str(archive_individual) + ".dat"
    parsed_file_list = read_tabdelimited(timefile)
    time_consumed =float(parsed_file_list[0][2])/TICKS_PER_SECOND #time_consumed = min(float(line[-1]), TICKS_PER_TRIAL)/TICKS_PER_SECOND
    if time_consumed > NUM_SECONDS:
        print(time_consumed)
    if estimate:
        performance = float(parsed_file_list[0][0])
    else:
        # look up that individual's performance in the faulty environment
        parsed_file_list = read_spacedelimited(path)

        performance = 0.0
        for item in parsed_file_list:
            indiv = item[0]
            if indiv == archive_individual:
                performance = float(item[-1])
    return performance, time_consumed

def get_best_performance_VE(path,faultpath,virtual_folder,bd_t,r,gener, until=None,estimate=True):
    # look up best observation of virtual energy
    obs = faultpath + virtual_folder + "/BO_output" + VE_tag + "/observations.dat"
    obs_list = read_spacedelimited(obs)
    i = 0
    best_VE = -float("inf")
    best_index = None
    for line in obs_list:
        if i > 0:  # ignore the first line
            number = float(line[-1])
            if number > best_VE:
                best_VE = number
                best_index = i  # ignore the first line
        i +=1
        if i > until:
            break
    return index2fullperformance(bd_t,path,faultpath,virtual_folder,best_index,r,gener, VE_tag,estimate )


def get_BO_development(bd_t, r, gener, path, faultpath, best_performances,time_lost, normal_folder,virtual_folder,virtual_energy,uniform,estimate):

    if virtual_energy:
        BOfile = faultpath + virtual_folder +"/BO_output"+VE_tag+"/best_observations.dat"
    elif uniform:
        BOfile = faultpath + normal_folder + "/BO_output" + uniform_tag + "/best_observations.dat"
    else:
        BOfile = faultpath + normal_folder + "/BO_output" + settings_tag + "/best_observations.dat"
    parsed_file_list = read_spacedelimited(BOfile)

    i=1
    time_cumulant=0.0
    x=[]
    y=[]
    count_full_eval=0
    for line in parsed_file_list:
        if i==1:
            i+=1
            continue # ignore the first line
        if virtual_energy:
            best_performance,time_consumed=get_best_performance_VE(path,faultpath,virtual_folder,bd_t,r,gener,until=i,estimate=estimate)
        else:
            best_performance=float(line[-1])
            time_consumed=NUM_SECONDS
        time_cumulant+=time_consumed
        x.append(time_cumulant)
        y.append(best_performance)
        i+=1
    time_lost = np.append(time_lost, x)
    best_performances = np.append(best_performances, y)
    return best_performances,  time_lost

def get_baseline_development(faultpath, title_tag, best_performances,time_lost):
    BOfile = faultpath + "/baselines/"+ title_tag + "/" + title_tag
    parsed_file_list = read_spacedelimited(BOfile)

    i=1
    time_cumulant=0.0
    x=[]
    y=[]
    count_full_eval=0
    best_performance=-float("inf")
    for line in parsed_file_list:
        performance=float(line[-1])
        if performance > best_performance:
            best_performance=performance
        time_consumed=NUM_SECONDS
        time_cumulant+=time_consumed
        x.append(time_cumulant)
        y.append(best_performance)
        i+=1
    time_lost = np.append(time_lost, x)
    best_performances = np.append(best_performances, y)
    return best_performances,  time_lost
def get_worker_developments(num_workers,bd_t, r, gener, path, faultpath, best_performances,time_lost, normal_folder,virtual_folder,virtual_energy,uniform):

    mean_time=[]
    mean_y=[]
    for worker in range(num_workers):
        BOfile = faultpath + normal_folder + "/BO_output" + VE_tag + "/async_stats_best"+str(worker)+".dat"
        parsed_file_list = read_spacedelimited(BOfile)
        i=1
        x=[]
        y=[]
        for line in parsed_file_list:
            best_performance=float(line[-1])
            time_cumulant=float(line[0])/(NUM_TRIALS*TICKS_PER_SECOND)
            x.append(time_cumulant)
            y.append(best_performance)
            i+=1
        mean_y=np.append(mean_y,y)
        mean_time=np.append(mean_time,x)
    time_lost = np.append(time_lost, mean_time)
    best_performances = np.append(best_performances, mean_y)
    return best_performances,  time_lost
def add_development_of_fault_performance(bd_t, r, gener, faultpath,
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
    if title_tag.startswith("BO"):
        return get_BO_development(bd_t, r, gener, path, faultpath, best_performances,time_lost, normal_folder,virtual_folder,virtual_energy,uniform,estimate)
    elif "single_exp" in title_tag:
        normal_folder+="/"+title_tag
        return get_worker_developments(int(NUM_AGENTS),bd_t, r, gener, path, faultpath, best_performances,time_lost, normal_folder,virtual_folder,virtual_energy,uniform)
        # try:
        #     lines=read_spacedelimited(faultpath+normal_folder+"/"+title_tag+"/BO_output"+VE_tag+"/fitness")
        #     fitness=float(lines[-1][0])
        #     return [fitness],  [3600.]
        # except Exception as e:
        #     print(e)
        #     return [],[]
    else:
        return get_baseline_development(faultpath + normal_folder, title_tag, best_performances,time_lost)


def add_fault_performance(bd_t, r, gener, nofaultperfs,best_nofaultperfs,maxindsnofault,faultpath,
                          best_performances,best_transfer,resilience, baseline=False,
                          title_tag="",virtual_energy=False):
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



def get_nofault_performances(nofaultpath,gener,runs):

        nofaultfilenames = [nofaultpath + str(run) + "/archive_" + str(gener) + ".dat" for run in runs]
        nofaultperfs = [np.array(list(get_ind_performances_uniquearchive(f).values())).flatten() for f in nofaultfilenames]
        max_nofaultperfs = [max(nofaultperfs[f]) for f in range(len(nofaultfilenames))]
        maxindsnofault = []
        for f in range(len(nofaultfilenames)):
            maxindnofault, best_performance = get_best_individual(nofaultfilenames[f], add_performance=True,
                                                                  index_based=True)
            maxindsnofault.append(maxindnofault)
            assert best_performance == max_nofaultperfs[f]
        return nofaultperfs,max_nofaultperfs,maxindsnofault

def significance_data(fitfuns,fitfunlabels,bd_type,runs,gener, by_faulttype=True, load_existing=False,title_tag="",virtual_energy=False):
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

    if virtual_energy:
        title_tag += "VE"

    loadfilename = "data/faulttype/summary_statistics_fault"+title_tag+".pkl" if by_faulttype else "data/combined/summary_statistics"+title_tag+".pkl"
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
            best_performance_data.append([[] for j in range(num_fault_types)])
            performance_data.append([[] for j in range(num_fault_types)])
            best_transfer_data.append([[] for j in range(num_fault_types)])
            transfer_data.append([[] for j in range(num_fault_types)])
            resilience_data.append([[] for j in range(num_fault_types)])
            recovery_data.append([[] for j in range(num_fault_types)])
            trial_data.append([[] for j in range(num_fault_types)])
            num_trials.append([[] for j in range(num_fault_types)])
            performance_loss_data.append([[] for j in range(num_fault_types)])

            BD_dir = datadir+"/Foraging"
                # get all the data from the archive: no fault

            nofaultpath=BD_dir + "/" + bd_type[i] + "/results"
            nofaultperfs, best_nofaultperfs, maxindsnofault = get_nofault_performances(nofaultpath,gener,runs)


            for fault in foraging_perturbations:
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
                                                                                                              title_tag=title_tag,virtual_energy=virtual_energy)



                    #otherwise transfer is undefined; we observe f=0 for some individuals in bordercoverage
                    # print(transfer.max())
                    # print(np.mean(transfer))


                    if by_faulttype:
                        faulttype,index=get_fault_type(fault)
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

    from scipy.stats import mannwhitneyu
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


        with open("results/fault/summary_table_faulttype"+title_tag,"w") as f:
            make_table(f,part_data,
                       rowlabels=foraging_fault_types,
                       columnlabels=legend_labels,
                       conditionalcolumnlabels=col_labels,
                       transpose=False)
        with open("results/fault/summary_table_faulttype_median"+title_tag,"w") as f:
            make_table(f,part_data,
                       rowlabels=foraging_fault_types,
                       columnlabels=legend_labels,
                       conditionalcolumnlabels=col_labels,
                       median=True,
                       transpose=False)
    else:
        if not load_existing:
            pickle.dump(all_data,
                        open(loadfilename, "wb"))
        part_data = [best_transfer_data, best_performance_data, resilience_data]
        with open("results/summary_table"+title_tag,"w") as f:
            make_table(f,part_data,
                       rowlabels=legend_labels,
                       columnlabels=[],
                       conditionalcolumnlabels=[("besttransfer","float2"),("bestperformance","float2"),("resilience","float2")])
        with open("results/summary_table_median"+title_tag, "w") as f:
            make_table(f, part_data,
                           rowlabels=legend_labels,
                           columnlabels=[],
                           conditionalcolumnlabels=[("besttransfer","float2"),("bestperformance","float2"),("resilience","float2")],
                            median=True)

def write_conditional(performance_list,index,file,max_reference,min_reference):
    U, p = ranksums(performance_list[0],performance_list[index])
    m_temp=np.mean(performance_list[index])
    if p < 0.05:
        if U > 0 :
            file.write("$\mathbf{%.2f}$ (+) "%(m_temp))
        else:
            file.write("$\mathbf{%.2f}$ (-) " % (m_temp))
    else:
        if U > 0:
            file.write("$%.2f$ " % (m_temp))
        else:
            file.write("$%.2f$ " % (m_temp))
    if index==0:
        ref=100*(m_temp/min_reference)
        file.write(" $(%.1f%%)$ &"%(ref))
    else:
        #ref=100*(m_temp/min_reference)
        file.write(" &")
def prepare_data(VE_tags, conditions, settings, max_evals,num_VE_conditions, gener, estimate, by_faulttype):
    # prepare the data for the two conditions
    best_performance_data = []
    time_loss = []
    percentage_eval_data = [[[] for fault in range(num_fault_types)] for j in range(num_VE_conditions)]
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
            best_performance_data.append([[[] for t in range(max_evals[c])] for j in range(num_fault_types)])
            time_loss.append([[[] for t in range(max_evals[c])] for j in range(num_fault_types)])
            BD_dir = datadir + "/Foraging"
            # get all the data from the archive: no fault

            nofaultpath = BD_dir + "/" + bd_type[i] + "/results"
            nofaultperfs, best_nofaultperfs, maxindsnofault = get_nofault_performances(nofaultpath, gener, runs)

            for fault in foraging_perturbations:
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
                        add_development_of_fault_performance(bd_type[i], r, gener, faultpath,
                                                             best_performances=[], time_lost=[], baseline=False,
                                                             title_tag=title_tag, virtual_energy=VE, uniform=uniform,
                                                             estimate=estimate)

                    if by_faulttype:
                        faulttype, index = get_fault_type(fault)
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
        pickle.dump(all_data, open("foraging_perturbation_results.pkl", "wb"))
    return all_data


def make_simple_table(best_performance_data,percentage_eval_data,time_loss,max_evals,conditions, plottag, reference_faultinjection_data, reference_performance_data, settings):
    # get the mean and sd for each fault category for this condition (BO or BO-VE)
    bd_index=0
    table30_file = open("performance_table"+plottag+"30.txt","w")

    for fault_category in range(num_fault_types):
        performances30 = [None for c in conditions]
        table30_file.write(foraging_fault_types[fault_category] + " & ")
        time = [[] for c in conditions]


        for c, condition in enumerate(conditions):
            # after equivalent of 30 evals
            t_3600 = None
            p_3600 = None
            p_sd_3600 = None
            mindist_3600 = float("inf")

            for t in range(max_evals[c]):
                data = best_performance_data[c][fault_category][t]/NUM_AGENTS
                mean = np.mean(data)
                sd = np.std(data)
                consumed = np.mean(time_loss[c][fault_category][t])

                if consumed >=29*NUM_SECONDS and consumed <=31*NUM_SECONDS: # try to find closest to 360
                    dist = abs(consumed - 30*NUM_SECONDS)
                    if dist < mindist_3600:
                        t_3600 = consumed
                        p_3600 = mean
                        p_sd_3600 = sd
                        mindist_3600 = dist
                        performances30[c] = data
            # print(str(t_1200) + " " + str(p_1200) + " " + str(p_sd_1200))
            # print(str(t_2400) + " " + str(p_2400) + " " + str(p_sd_2400))
            print(str(t_3600) + " " + str(p_3600) + " " + str(p_sd_3600))
            if c==len(conditions) - 1:
                table30_file.write(r"  $%.3f \pm %.2f$ \\\\"%(np.mean(performances30[c]),np.std(performances30[c])))
            else:
                table30_file.write(r"  $%.3f \pm %.2f$ &")
        table30_file.write("\n")



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

    for fault_category in range(num_fault_types):
        percentage.append([])
        performances30 = [None for c in conditions]
        # performances10 =  [None for c in conditions]
        # performances20 =  [None for c in conditions]
        performances1 = [None for c in conditions]
        mean_lines = [[] for c in conditions]
        sd_lines1 = [[] for c in conditions]
        sd_lines2 = [[] for c in conditions]
        min_reference = np.mean(reference_faultinjection_data[bd_index][fault_category])/NUM_AGENTS
        max_reference = np.mean(reference_performance_data[bd_index][fault_category])/NUM_AGENTS
        colors = ["C8", "C0","C1","C2", "C3","C4", "C5","C6","C7"]  # colors for the lines
        # (numsides, style, angle)
        markers = ["*", "o","D","X","v","+", "$\dagger$","^","$\spadesuit$"]  # markers for the lines

        percentage_file.write(foraging_fault_types[fault_category] + " & ")
        table30_file.write(foraging_fault_types[fault_category] + " & ")
        # table10_file.write(foraging_fault_types[fault_category] + " & ")
        # table20_file.write(foraging_fault_types[fault_category] + " & ")
        #table1_file.write(foraging_fault_types[fault_category] + " & ")
        # add the exhaustive search performance as a maximum
        time = [[] for c in conditions]


        for c, condition in enumerate(conditions):
            tag, VE, VE_tag_index = settings[c]
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
                if max_evals[c] == 1 or (consumed >=29*NUM_SECONDS and consumed <=31*NUM_SECONDS): # try to find closest to 360
                    dist = abs(consumed - 30*NUM_SECONDS)
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
                elif VE and consumed >= 3600.0:
                    final_performances=np.append(final_performances,mean - min_reference)
                    break
            # print(str(t_1200) + " " + str(p_1200) + " " + str(p_sd_1200))
            # print(str(t_2400) + " " + str(p_2400) + " " + str(p_sd_2400))
            print(str(t_3600) + " " + str(p_3600) + " " + str(p_sd_3600))



            #table_file.write("$%.2f \pm %.2f$ & $%.2f \pm %.2f$ &"%(p_360,p_sd_360,p_2400,p_sd_2400))
            table30_file.write("$%.2f \pm %.2f$  &"%(p_3600,p_sd_3600))

        table30_file.write("\n")
        percentage_file.write("\n")

        # for c, condition in enumerate(conditions):
        #     write_conditional(performances1, c, table1_file, max_reference, min_reference)
        #     write_conditional(performances10,c,table10_file,max_reference,min_reference)
        #     write_conditional(performances20, c, table20_file,max_reference,min_reference)
        #     write_conditional(performances30, c, table30_file,max_reference,min_reference)
        # # table10_file.write("$\mathbf{%.2f}$ (+) &" % ((np.mean(performances10[0]) - min_reference)/(max_reference - min_reference)))
        # # table20_file.write("$\mathbf{%.2f}$ (+) &" % ((performances20[0] - min_reference)/(max_reference - min_reference)))
        # # table30_file.write("$\mathbf{%.2f}$ (+) &" % ((performances30[0] - min_reference)/(max_reference - min_reference)))
        # table1_file.write("\n")
        # table10_file.write("\n")
        # table20_file.write("\n")
        # table30_file.write("\n")
        additional_lines = [(time[0], [min_reference for t in time[0]]), (time[0], [max_reference for t in time[0]])]
        createPlot(mean_lines, x_values=time,
                   save_filename="recovery_fault_"+str(foraging_fault_types[fault_category])+plottag+".pdf", legend_labels=conditions,
                   colors=colors, markers=markers, xlabel="Time ($s$)",
                   ylabel="Best performance",
                   xlim=[0, 4000], xscale="linear", yscale="linear", ylim=[0,max_reference+1],
                   legendbox=None, annotations=[], xticks=[], yticks=[], task_markers=[], scatter=False,
                   legend_cols=1, legend_fontsize=26, legend_indexes=[], additional_lines=additional_lines, index_x=[],
                   xaxis_style="plain", y_err=[], force=True) #, fill_between=(sd_lines1, sd_lines2))
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
    if comparison=="baselines":
        conditions = ["SRBO","SRBO-Uniform",
                      "Random", "Gradient-ascent"]
        settings = [("BO", False, None), ("BO",False,None),
                   ("random", False, None), ("gradient_closest", False, None)]
        plottag="ALL"
        VE_tags = ["_VE_init" + str(j) for j in [3, 4, 5, 6, 8]]

        num_VE_conditions=4
    elif comparison=="heterogeneous":
        conditions = ["H-SRBO-Local","H-SRBO (Known fault)","H-SRBO","H-SRBO (Random identification)"]
        settings = [("single_exp", False, "localpenalisation"),("single_exp_known", False, "reset_nocollisionstop"),
                    ("single_exp", False, "reset_nocollisionstop"),("single_exp_random", False, "reset_nocollisionstop"),
                    ("single_exp_randomsearch", False, "reset_nocollisionstop")]
        plottag="HETEROGENEOUS"
        VE_tags = ["_VE_init" + str(j) for j in [3, 4, 5, 6, 8]]
        num_VE_conditions=5
    elif comparison=="fest":
        conditions = ["SRBO", "VE-SRBO E(0)=3","VE-SRBO E(0)=4","VE-SRBO E(0)=5","VE-SRBO E(0)=6","VE-SRBO E(0)=8"]
        settings = [("BO", False, None), ("BO", True, 0), ("BO", True, 1), ("BO", True, 2), ("BO", True, 3),("BO", True, 4)]
        plottag="fest_params"
        VE_tags = ["_nocollision_init" + str(j) for j in [3, 4, 5, 6, 8]]
        num_VE_conditions = 5
    elif comparison=="VE":
        conditions = ["SRBO", "VE-SRBO E(0)=5", "VE-SRBO E(0)=10"]
        settings = [("BO", False, None), ("BO", True, 0),("BO", True, 1)]
        plottag="VE_params"
        num_VE_conditions = 5
        VE_tags = ["_init" + str(j) + "_single" for j in [5,10]]
    else:
        raise Exception()

    if estimate:
        plottag+="estimate"
    if from_file:
        best_performance_data, time_loss, percentage_eval_data = pickle.load(open("foraging_perturbation_results.pkl","rb"))
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
#     for f in range(num_fault_types):
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
    conditions = ["SRBO", "VE-SRBO E(0)=3", "VE-SRBO E(0)=4", "VE-SRBO E(0)=5", "VE-SRBO E(0)=6", "VE-SRBO E(0)=8",
                  "Random", "Gradient-ascent"]
    settings = [("BO", False, None), ("BO", True, 0), ("BO", True, 1), ("BO", True, 2), ("BO", True, 3),
                ("BO", True, 4), ("random", False, None), ("gradient_closest", False, None)]
    num_VE_conditions = 5
    best_performance_data, time_loss, percentage_eval_data = pickle.load(open("foraging_perturbation_results.pkl", "rb"))
    percentage = []
    for fault_category in range(num_fault_types):

        for c, condition in enumerate(conditions):
            tag, VE, VE_tag_index = settings[c]
            for t in range(max_evals[c]):
                if np.mean(time_loss[c][fault_category][t]) >= 3600.0:
                    p = best_performance_data[c][fault_category][t]
                    final_performances = np.append(final_performances, p)
                    break



def make_significance_table(fitfunlabels,conditionlabels,qed_index,table_type="resilience"):

    best_performance_data, performance_data, best_transfer_data, transfer_data, resilience_data = pickle.load(
            open("data/fitfun/summary_statistics_fitfun.pkl", "rb"))
    if table_type=="resilience":
        qed=resilience_data[qed_index] # QED
        data=resilience_data
    else:
        for i, fitfun in enumerate(fitfunlabels):
            for j in range(len(best_performance_data)):
                best_performance_data[j][i]/=baseline_performances[fitfun]
        qed=best_performance_data[qed_index]
        data=best_performance_data
    with open("results/fault/table/significance_table"+table_type,"w") as f:
        f.write(r"& \multicolumn{9}{c}{\textbf{Condition}}")
        newline_latex(f,add_hline=True)
        f.write(r"\textbf{Swarm task}")
        f.write(r"& QED")
        for condition in conditionlabels:
            if condition != "QED":
                f.write(r"& \multicolumn{3}{c|}{"+str(condition)+"}")
        newline_latex(f,add_hline=True)
        f.write(r"& %s " % (table_type))
        for condition in conditionlabels:
            if condition != "QED":
                f.write(r"& %s & significance & effect"%(table_type))
        newline_latex(f,add_hline=True)

        m=len(fitfuns)*3 # number of comparisons
        alpha_weak=.05/float(m)
        print("will use alpha=" + str(alpha_weak))
        alpha_best=.001/float(m) #
        print("will use alpha="+str(alpha_best))
        for k,fitfun in enumerate(fitfunlabels):
            f.write(fitfun)
            x = qed[k]
            # first write QED's resilience score
            mx = np.median(x)
            iqrx = np.quantile(x, 0.75) - np.quantile(x, 0.25)
            f.write(r"& $%.2f \pm %.1f$" % (mx,iqrx))
            # now write other's resilience score and significance values
            for j, condition in enumerate(conditionlabels):
                if condition == "QED":
                    continue
                y = data[j][k]
                m=np.median(y)
                iqr=np.quantile(y,0.75) - np.quantile(y,0.25)
                U, p = ranksums(x, y)
                p_value = "p=%.3f"%(p) if p>0.001 else r"p<0.001"
                if p < alpha_best:
                    p_value+="^{**}"
                else:
                    if p < alpha_weak:
                        p_value+="^{*}"
                delta,label = cliffs_delta(U, x, y)
                delta_value = r"\mathbf{%.2f}"%(delta) if label == "large" else r"%.2f"%(delta)
                f.write(r"& $%.2f \pm %.1f$ & $%s$ & $%s$"%(m,iqr,p_value,delta_value))
            newline_latex(f)

def make_significance_table_compact(fitfunlabels,conditionlabels,qed_index,table_type="resilience"):

    best_performance_data, performance_data, best_transfer_data, transfer_data, resilience_data = pickle.load(
            open("data/fitfun/summary_statistics_fitfun.pkl", "rb"))
    if table_type=="resilience":
        qed=resilience_data[qed_index] # QED
        data=resilience_data
    else:
        for i, fitfun in enumerate(fitfunlabels):
            for j in range(len(best_performance_data)):
                best_performance_data[j][i]/=baseline_performances[fitfun]
        qed=best_performance_data[qed_index]
        data=best_performance_data
    with open("results/fault/table/significance_table_compact"+table_type,"w") as f:
        f.write(r"& \multicolumn{6}{c}{\textbf{Condition}}")
        newline_latex(f,add_hline=True)
        f.write(r"\textbf{Swarm task}")
        f.write(r"& QED")
        for condition in conditionlabels:
            if condition != "QED":
                f.write(r"& \multicolumn{2}{c|}{"+str(condition)+"}")
        newline_latex(f,add_hline=True)
        f.write(r"& %s " % (table_type))
        for condition in conditionlabels:
            if condition != "QED":
                f.write(r"& effect & significance "%(table_type))
        newline_latex(f,add_hline=True)

        m=len(fitfuns)*3 # number of comparisons
        alpha_weak=.05/float(m)
        print("will use alpha=" + str(alpha_weak))
        alpha_best=.001/float(m) #
        print("will use alpha="+str(alpha_best))
        for k,fitfun in enumerate(fitfunlabels):
            f.write(fitfun)
            x = qed[k]
            # now write other's resilience score and significance values
            for j, condition in enumerate(conditionlabels):
                if condition == "QED":
                    continue
                y = data[j][k]
                U, p = ranksums(x, y)
                p_value = "p=%.3f"%(p) if p>0.001 else r"p<0.001"
                if p < alpha_best:
                    p_value+="^{**}"
                else:
                    if p < alpha_weak:
                        p_value+="^{*}"
                delta,label = cliffs_delta(U, x, y)
                delta_value = r"\mathbf{%.2f}"%(delta) if label == "large" else r"%.2f"%(delta)
                f.write(r" & $%s$ & $%s$"%(delta_value,p_value))
            newline_latex(f)


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
def get_max_performances(bd_type,fitfuns,generation):
    maximum={fitfun: 0.0 for fitfun in fitfuns}
    for j in range(len(fitfuns)):
        for i in range(len(bd_type)):

            print(fitfuns[j])
            BD_dir = datadir + "/Foraging"
            # get all the data from the archive: no fault

            nofaultpath = BD_dir + "/" + bd_type[i] + "/results"

            nofaultperfs,max_nofaultperfs,maxindsnofault = get_nofault_performances(nofaultpath,generation,runs)
            max_performance = max(max_nofaultperfs)

            if max_performance>maximum[fitfuns[j]]:
                maximum[fitfuns[j]]=max_performance


    pickle.dump(maximum,open("data/fitfun/foraging_maximal_fitness.pkl","wb"))

def determine_noise():
    BD_dir = datadir + "/Foraging"
    # get all the data from the archive: no fault

    nofaultpath = BD_dir + "/history/results"
    nofaultperfs, best_nofaultperfs, maxindsnofault = get_nofault_performances(nofaultpath, "20000", runs)

    v = [np.var(p) for p in nofaultperfs]
    print(v)

if __name__ == "__main__":
    # significance_data(fitfuns, fitfunlabels, bd_type, runs, generation, by_faulttype=True, load_existing=False,
    #                  title_tag="",virtual_energy=False)
    # significance_data(fitfuns, fitfunlabels, bd_type, runs, generation, by_faulttype=True, load_existing=False,
    #                  title_tag="BO",virtual_energy=False)
    #determine_noise()
    significance_data(fitfuns, fitfunlabels, bd_type, runs, generation, by_faulttype=True, load_existing=False,
                     title_tag="",virtual_energy=False)
    #development_data(bd_type, runs, 20000, by_faulttype=True, max_evals=[30,100,100,100,100,100],from_file=True,comparison="VE",estimate=False)
    #development_data(bd_type, runs, 20000, by_faulttype=True, max_evals=[30, 100, 100, 100, 100, 100], from_file=False,comparison="fest", estimate=False)
    #development_data(bd_type, runs, 20000, by_faulttype=True, max_evals=[30, 100, 100, 100, 100, 100], from_file=False,comparison="fest", estimate=True)


    development_data(bd_type, runs, 20000, by_faulttype=True, max_evals=[30,30,30,30],from_file=False,comparison="baselines",estimate=False)
    development_data(bd_type, runs, 20000, by_faulttype=True, max_evals=[30,30,30,30],from_file=False,comparison="heterogeneous",estimate=False)



