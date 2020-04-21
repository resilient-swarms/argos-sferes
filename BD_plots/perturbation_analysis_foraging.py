from sklearn.neighbors import KernelDensity

from dimensionality_plot import *
from perturbance_metrics import *

from foraging_faults import *
from BD_metrics import *
from NCD import *
from reduce_translated_archive import *
HOME_DIR = os.environ["HOME"]
RESULTSFOLDER="results"
from plots import *

from significance import *
import pickle


NUM_SECONDS=120.0
TICKS_PER_SECOND=5.0
TICKS_PER_TRIAL=NUM_SECONDS*TICKS_PER_SECOND

baseline_performances = pickle.load(open("data/fitfun/foraging_maximal_fitness.pkl", "rb"))

runs=range(1,6)
bd_type = ["history"]  # legend label
legend_labels = ["HBD"]  # labels for the legend
fitfuns = ["Foraging"]
fitfunlabels = [""]


colors = ["C" + str(i) for i in range(len(bd_type))]
markers = [(2, 1, 0), (3, 1, 0), (2, 1, 1), (3, 1, 1)]

datadir = HOME_DIR + "/Data/"
generation = "20000"
history_type = "xy"





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


def index2fullperformance(bd_t,path,faultpath, virtual_folder, best_index, r, gener):
    # get the corresponding BD
    samp = faultpath + virtual_folder + "/BO_output/samples.dat"
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
    # look up that individual's performance in the faulty environment
    parsed_file_list = read_spacedelimited(path)

    performance=0.0
    for item in parsed_file_list:
        indiv = item[0]
        if indiv == archive_individual:
            performance = float(item[-1])
    # if performance is None:
    #     raise Exception("individual not found in faulty archive")
    timefile = faultpath + virtual_folder + "/BO_output/fitness" + str(archive_individual) + ".dat"
    parsed_file_list = read_tabdelimited(timefile)
    time_consumed = min(float(parsed_file_list[0][1]),TICKS_PER_TRIAL)/TICKS_PER_SECOND #time_consumed = min(float(line[-1]), TICKS_PER_TRIAL)/TICKS_PER_SECOND

    return performance, time_consumed


def get_best_performance_VE(path,faultpath,virtual_folder,bd_t,r,gener):
    # look up best observation of virtual energy
    obs = faultpath + virtual_folder + "/BO_output/observations.dat"
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
    return index2fullperformance(bd_t,path,faultpath,virtual_folder,best_index,r,gener)

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

        if title_tag=="BO":

            if virtual_energy:
                BOfile = faultpath + virtual_folder +"/BO_output/best_observations.dat"
            else:
                BOfile = faultpath + normal_folder + "/BO_output/best_observations.dat"
            parsed_file_list = read_spacedelimited(BOfile)

            if virtual_energy:
                best_performance,_=get_best_performance_VE(path,faultpath,virtual_folder,bd_t,r,gener)
            else:
                last_line = parsed_file_list[-1]
                best_performance=float(last_line[-1])
            num_trials=len(parsed_file_list) - 1 # count the lines, but top line does not count

            if virtual_energy:
                BOfile = faultpath + virtual_folder + "/BO_output/observations.dat"
            else:
                BOfile = faultpath + normal_folder + "/BO_output/observations.dat"
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
                    performance, time_consumed = index2fullperformance(bd_t,path,faultpath,virtual_folder,i,r,gener)
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



    loadfilename = "data/fitfun/summary_statistics_fault"+title_tag+".pkl" if by_faulttype else "data/combined/summary_statistics"+title_tag+".pkl"
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
            best_performance_data.append([[] for i in range(num_fault_types)])
            performance_data.append([[] for i in range(num_fault_types)])
            best_transfer_data.append([[] for i in range(num_fault_types)])
            transfer_data.append([[] for i in range(num_fault_types)])
            resilience_data.append([[] for i in range(num_fault_types)])
            recovery_data.append([[] for i in range(num_fault_types)])
            trial_data.append([[] for i in range(num_fault_types)])
            num_trials.append([[] for i in range(num_fault_types)])
            performance_loss_data.append([[] for i in range(num_fault_types)])

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
                        #resilience_data[i][index] = np.append(resilience_data[i][index],resilience)
                        trial_data[i][index].append(_trial_time)
                        performance_loss_data[i][index].append(_performance_loss)
                        num_trials[i][index].append(_num_trials)
                    else:
                        # normalise by maximal fitness (make different performance data comparable)
                        best_performance_data[i] = np.append(best_performance_data[i],best_performances/baseline_performances[fitfuns[j]])
                        best_transfer_data[i] = np.append(best_transfer_data[i],best_transfer)
                        #resilience_data[i] = np.append(resilience_data[i],resilience)
                        trial_data[i].append(_trial_time)
                        num_trials[i].append(_num_trials)

    from scipy.stats import mannwhitneyu
    all_data = (best_performance_data, performance_data, best_transfer_data, transfer_data,resilience_data)

    if by_faulttype:
        if not load_existing:
            pickle.dump(all_data,
                        open(loadfilename, "wb"))

        # for i in range(len(best_performance_data)):
        #     for j in range(len(fitfuns)):
        #         best_performance_data[i][j]/=baseline_performances[fitfuns[j]]
        if title_tag=="BO":
            part_data = (best_performance_data, trial_data, performance_loss_data, num_trials)
            col_labels =  [("Recovery performance", "float2"),("Evaluation time ($s$)", "float2"),  ("Search loss", "float2"),("Number of evaluations","float2") ]
        else:
            part_data = (best_transfer_data,best_performance_data)
            col_labels=[("Fault injection performance","float2"),("Recovery performance","float2")]

        if virtual_energy:
            title_tag+="VE"
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


def test_significance(bd_type,by_fitfun,data_type):

   if not by_fitfun:
       best_performance_data, performance_data, best_transfer_data,transfer_data, recovery_data, resilience_data = pickle.load(open("data/combined/summary_statistics.pkl", "rb"))
   else:
       best_performance_data, performance_data, best_transfer_data , transfer_data, resilience_data = pickle.load(open("data/fitfun/summary_statistics_fitfun.pkl", "rb"))

   if data_type=="resilience":
       data=resilience_data
   elif data_type == "recovery":
       data = recovery_data
   elif data_type=="best_performance":
       data=best_performance_data
   elif data_type=="best_transfer":
       data=best_transfer_data
   elif data_type == "transfer":
       data = transfer_data
   elif data_type == "performance":
       data = performance_data
   else:
       raise Exception("specify data_type")

   if by_fitfun:
       for f in range(len(fitfuns)):
           print(fitfuns[f])
           for i in range(len(bd_type)):
               x=data[i][f]
               for j in range(0,len(bd_type)):
                   y=data[j][f]
                   stat,p = ranksums(x,y)
                   print("%s vs %s : U=%.2f, p=%.6f"%(bd_type[i],bd_type[j],stat,p))
                   if i != j:
                       delta, label= cliffs_delta(stat,x,y)
                       print("Cliffs delta: %.3f   %s"%(delta,label))
   else:
       print("OVERALL")
       for i in range(len(bd_type)):
           x = data[i]
           for j in range(0, len(bd_type)):
               y = data[j]
               stat, p = ranksums(x, y)
               print("%s vs %s : U=%.2f, p=%.6f" % (bd_type[i], bd_type[j], stat, p))
               if i != j:
                   delta, label = cliffs_delta(stat, x, y)
                   print("Cliffs delta: %.3f   %s"%(delta,label))
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



if __name__ == "__main__":
    # significance_data(fitfuns, fitfunlabels, bd_type, runs, generation, by_faulttype=True, load_existing=False,
    #                  title_tag="",virtual_energy=False)
    # significance_data(fitfuns, fitfunlabels, bd_type, runs, generation, by_faulttype=True, load_existing=False,
    #                  title_tag="BO",virtual_energy=False)
    significance_data(fitfuns, fitfunlabels, bd_type, runs, generation, by_faulttype=True, load_existing=False,
                     title_tag="BO",virtual_energy=True)


