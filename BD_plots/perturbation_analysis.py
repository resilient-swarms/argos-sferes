from sklearn.neighbors import KernelDensity

from dimensionality_plot import *
from perturbance_metrics import *
from NCD import *
from reduce_translated_archive import *
HOME_DIR = os.environ["HOME"]
RESULTSFOLDER="results"
from plots import *

from scipy.stats import *
import pickle

baseline_performances = pickle.load(open("data/fitfun/maximal_fitness.pkl", "rb"))

faults = range(50)
F = len(faults)
runs = range(1, 6)
bd_type = ["history", "Gomes_sdbc_walls_and_robots_std", "cvt_rab_spirit","environment_diversity"]  # legend label
legend_labels = ["HBD", "SDBC", "SPIRIT", "QED"]  # labels for the legend
fitfuns = ["Aggregation","Dispersion","DecayCoverage","DecayBorderCoverage","Flocking"]
fitfunlabels = ["Aggregation","Dispersion","DecayCoverage","DecayBorderCoverage","Flocking"]
# get_max_performances(bd_type, fitfuns,"30000")

colors = ["C" + str(i) for i in range(len(bd_type))]
markers = [(2, 1, 0), (3, 1, 0), (2, 1, 1), (3, 1, 1)]

datadir = HOME_DIR + "/Data/"
generation = "30000"
history_type = "xy"


def gather_perturbation_data(BD_DIRECTORY,generation,faults, runs, get_NCD=True,
                             history_type="sa",translation_type="handcrafted",centroids=[],self_dist_type=None):
    """
    use existing state/observation-action trajectory files and calculate pair-wise NCDs for all individuals within the sa;e run
    the average is returned
    :return:
    """
    ncds = []
    performances = []
    performance_comps = []
    dists = [] # distance to normal in the projected space
    categories = []
    self_dists = []  # distance to normal in the own space


    for fault in faults:
        for run in runs:
            nofault_dir = BD_DIRECTORY + "/FAULT_NONE/results" + str(run)
            self_dir = BD_DIRECTORY + "/results"+str(run)
            history_comp, performance_comp, bd_comp, self_bd_comp = get_help_data(nofault_dir, generation,
                                                                                  history_type, translation_type,
                                                                                  self_dir=self_dir)
            history_file, performance, bd,self_bd = \
                get_help_data(BD_DIRECTORY+"/faultyrun"+str(run)+"_p"+str(fault)+"/results"+str(run),generation,history_type,translation_type,self_dir=self_dir)
            if get_NCD:
                # get ncd
                ncd = NCD(history_comp,history_file,perform_lzma, from_zip=True)
                ncds.append(ncd)
            else:
                ncds=None
            performances.append(performance)
            performance_comps.append(performance_comp)
            if translation_type=="spirit":
                dists.append(avg_variation_distance(bd_comp,bd,num_actions=16))
            else:
                dists.append(norm_Euclidian_dist(bd_comp,bd))

            self_dists.append(self_dist_type(self_bd_comp,self_bd))

            if centroids:
                index,centr=transform_bd_cvtmapelites(bd, centroids)
                categories.append(index)



    return ncds, performances, performance_comps, dists, categories, self_dists

def gather_unperturbed_diversity(BD_DIRECTORY,generation,faults, runs, get_NCD=True,
                             history_type="sa",translation_type="handcrafted",centroids=[],self_dist_type=None):
    """
    gather BD data from the evolutionary run's archive directly
    :return:
    """

    dists = []
    categories = []
    self_dists = []

    for fault in faults:

        for run in runs:
            self_dir=BD_DIRECTORY + "/results" + str(run)
            # get projected and unprojected behavioural descriptor
            history_comp, performance_comp, bd_comp, self_bd_comp = get_help_data(
                BD_DIRECTORY + "/FAULT_NONE/results" + str(run), generation,
                history_type, translation_type, self_dir=self_dir)
            # get projected and unprojected behavioural descriptor
            bd, self_bd = get_help_data_unperturbed(BD_DIRECTORY,
                                               "/faultyrun"+str(run)+"_p"+str(fault)+"/results"+str(run),
                                               "/FAULT_NONE/results" + str(run),
                                               generation="30000",
                                               translation_type=translation_type,
                                                self_dir=self_dir
                                                )


            if translation_type=="spirit":
                dists.append(avg_variation_distance(bd_comp,bd,num_actions=16))
            else:
                dists.append(norm_Euclidian_dist(bd_comp,bd))
            if centroids:
                index,centr=transform_bd_cvtmapelites(bd, centroids)
                categories.append(index)

            if self_dist_type is not None:
                self_dists.append(self_dist_type(self_bd_comp,self_bd))

    return dists, categories, self_dists



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

def get_pca(data_path, runs, archive_file_path,bd_labels,components=2):
    bd = np.array(get_combined_archive(data_path, runs, archive_file_path,by_bin="list",include_val=False))
    df = pd.DataFrame(bd, columns=bd_labels)
    pca = PCA(n_components=components)
    pca.fit(df[bd_labels].values)

def get_pca_result(pca,bd,bins=3):
    pca_result = pca.transform(bd)
    minima = np.min(pca_result,axis=0) # get the minimum for each dimension
    maxima = np.max(pca_result,axis=0) # get the maximum for each dimension
    bin_sizes = (maxima - minima) / float(bins)

    bd_cat = []
    for datapoint in pca_result:
        bd_cat.append(bin_single_point(datapoint, minima, bins, bin_sizes))

    return bd_cat



def scatter_plot(x,y,colors,area,filename):
    plt.scatter(x, y, s=area, c=colors, alpha=0.5)
    plt.savefig(filename)

def descriptor_perturbation_plot(data_path, runs, archive_file_path, bd_labels):
    """
    scatterplot of the phenotypes of solutions after perturb. as a function of the drop in fitness by the perturb.
    :return:
    """
    # get all datapoints' corresponding categories
    categories = bd_categories(data_path, runs, archive_file_path, bd_labels, components=2)

def get_delta_P(non_perturbed_path,perturbed_path,max):
    _index, np_performance = get_best_individual(non_perturbed_path,add_performance=True)
    _index, p_performance = get_best_individual(perturbed_path,add_performance=True)
    return p_performance - np_performance


def gather_perturbation_results(datadir,generation,bd_type,fitfuns,faults,runs,history_type,perturbed=True):
    centroids_sdbc=load_centroids("centroids/centroids_10_10.dat")
    centroids=load_centroids("centroids/centroids_10_3.dat")

    def mv(p1, p2):
        return avg_variation_distance(p1, p2, 16)

    dists = {"history": norm_Euclidian_dist, "Gomes_sdbc_walls_and_robots_std": norm_Euclidian_dist, "cvt_rab_spirit":mv,"environment_diversity": norm_Euclidian_dist}

    for bd in bd_type:
        for fitfun in fitfuns:
            title = fitfun + "range0.11"
            prefix = datadir + "/" + title + "/" + bd

            #
            if perturbed:
                # ncds, performances, nofaultperfs, euclids, categories= gather_perturbation_data(prefix, generation, faults,
                #                                                                     runs=runs, history_type=history_type,
                #
                #                                                                   translation_type="sdbc",centroids=centroids_sdbc,get_NCD=False)

                ncds, performances, nofaultperfs, maxvars,categories, self_dists= gather_perturbation_data(prefix, generation, faults,
                                                                   runs=runs, history_type=history_type,
                                                                   translation_type="spirit",get_NCD=False,self_dist_type=dists[bd])

                # _, _, _ , _, categories_handcrafted = gather_perturbation_data(prefix, generation, faults,
                #                                                                     runs=runs, history_type=history_type,
                #                                                                     translation_type="handcrafted",centroids=centroids,get_NCD=False)



                dp_file,ncd_file, euclid_file, maxvars_file, category_file, category_h_file,selfdist_file = filenames(fitfun,bd,history_type)
                pickle.dump((performances,nofaultperfs), open(dp_file, "wb"))
                pickle.dump(ncds, open(ncd_file, "wb"))
                pickle.dump(maxvars, open(maxvars_file, "wb"))
                pickle.dump(categories, open(category_file, "wb"))
                pickle.dump(self_dists, open(selfdist_file, "wb"))
                #pickle.dump(categories_handcrafted, open(category_h_file, "wb"))
            else:

                # _,categories_handcrafted = gather_unperturbed_diversity(prefix,generation, faults, runs,
                #                                                         get_NCD=False,
                #                                                         history_type=history_type,
                #                                                         translation_type="handcrafted", centroids=centroids
                #                                                         )
                # euclids,categories = gather_unperturbed_diversity(prefix,generation, faults, runs,
                #                                                   get_NCD=False,
                #                                                   history_type=history_type,
                #                                                   translation_type="sdbc", centroids=centroids_sdbc
                #                                                  )

                maxvars, categories, self_dists= gather_unperturbed_diversity(prefix,generation, faults, runs,
                                                          get_NCD=False,
                                                          history_type=history_type,
                                                          translation_type="spirit", centroids=[],self_dist_type=dists[bd]
                                                          )

                euclid_file, maxvar_file, category_file, categoryh_file,selfdist_file = unperturbed_filenames(fitfun,bd,history_type)
                #pickle.dump(euclids, open(euclid_file, "wb"))
                pickle.dump(maxvars, open(maxvar_file, "wb"))
                pickle.dump(categories, open(category_file, "wb"))
                pickle.dump(self_dists, open(selfdist_file, "wb"))
                #pickle.dump(categories_handcrafted, open(categoryh_file, "wb"))
def gather_category_results(bd_type, fitfuns, faults, runs):
    for bd in bd_type:
        data_dir = HOME_DIR + "/DataFinal/datanew"

        categories_list = []
        delta_p_list = []

        for fitfun in fitfuns:
            title = fitfun + "range0.11"
            prefix = data_dir + "/" + title + "/" + bd
            pca = get_pca(prefix,[1],"archive_1000.dat",["SDBC"+str(i) for i in range(10)])

            for run in runs:
                for fault in faults:
                    maxind, maxf, maxbd = get_best_individual(prefix + "/results" + str(run) + "/analysis_sdbc.dat")
                result = get_pca_result(pca,[maxbd],bins=3)

                pickle.dump(ncds_list, open(ncd_file, "wb"))
def filenames(fitfun,bd,history_type):
    #dp_file, ncd_file, euclid_file, ent_file, category_file
    prefix= "data/fitfun/"+fitfun+"/"+ bd + history_type
    return prefix + "_DeltaPs.pkl",prefix +"_ncds.pkl",\
           prefix +"_euclids.pkl",prefix +"_maxvars.pkl", \
          prefix +  "_categories.pkl", prefix +  "handcrafted_categories.pkl", prefix+"selfdists.pkl"
def unperturbed_filenames(fitfun,bd,history_type):
    #dp_file, ncd_file, euclid_file, ent_file, category_file
    prefix= "data/fitfun/"+fitfun+"/"+ bd + history_type
    return prefix +"_euclidsunperturbed.pkl",prefix +"_maxvarsunperturbed.pkl", \
          prefix +  "_categoriesunperturbed.pkl", prefix +  "handcrafted_categoriesunperturbed.pkl", prefix+"selfdistsunperturbed.pkl"


def get_baseline_performances(nofaultpath):
        nofaultfilenames = [nofaultpath + str(run) + "/fitness" for run in runs]
        nofaultperfs = [get_baseline_fitness(f) for f in
                        nofaultfilenames]

        return nofaultperfs

def add_fault_performance(j, r, gener, nofaultperfs,best_nofaultperfs,maxindsnofault,faultpath,
                          best_performances,performances,best_transfer,transfer,recovery,resilience, baseline=False, add_faultperf=False):
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
    path=faultpath+"/fitness" if baseline else faultpath+"/analysis" + str(gener) + "_handcrafted.dat"
    if baseline:

        best_performance=get_baseline_fitness(path)
    else:
        temp = np.array(list(get_ind_performances_uniquearchive(path).values())).flatten()
        performances = np.append(performances, temp)

        maxind, best_performance = get_best_individual(path, add_performance=True, index_based=True)

        # all performances vs all nofaultperformances
        for k in range(len(nofaultperfs[r])):
            transfer = np.append(transfer, [(temp[k] - nofaultperfs[r][k]) / baseline_performances[fitfuns[j]]])  # avoid NANs

        best_transfer = np.append(best_transfer,
                                  (temp[maxindsnofault[r]] - best_nofaultperfs[r]) / best_nofaultperfs[r])
        if add_faultperf:
            best_faultperf=temp[maxindsnofault[r]]

    best_performances = np.append(best_performances, best_performance)

    recovery = np.append(recovery, [(best_performance - best_nofaultperfs[r]) / baseline_performances[
        fitfuns[j]]])  # best performance vs best nofaultperf
    resilience = np.append(resilience, (best_performance - best_nofaultperfs[r]) / best_nofaultperfs[r])
    if add_faultperf:
        return best_performances, performances, best_transfer, transfer, recovery, resilience, best_faultperf
    else:
        return best_performances, performances, best_transfer, transfer, recovery,resilience


def get_nofault_performances(nofaultpath,gener,runs):

        nofaultfilenames = [nofaultpath + str(run) + "/analysis" + str(gener) + "_handcrafted.dat" for run in runs]
        nofaultperfs = [np.array(list(get_ind_performances_uniquearchive(f).values())).flatten() for f in nofaultfilenames]

        best_nofaultperfs = np.array([get_performance_data(nofaultpath + str(run), gener) for run in
                                      runs])
        maxindsnofault = []
        for f in range(len(nofaultfilenames)):
            maxindnofault, best_performance = get_best_individual(nofaultfilenames[f], add_performance=True,
                                                                  index_based=True)
            maxindsnofault.append(maxindnofault)
            assert best_performance == best_nofaultperfs[f]
        return nofaultperfs,best_nofaultperfs,maxindsnofault

def significance_data(fitfuns,fitfunlabels,bd_type,runs,faults,gener, by_fitfun=True, load_existing=False,title_tag=""):
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



    loadfilename = "data/fitfun/summary_statistics_fitfun"+title_tag+".pkl" if by_fitfun else "data/combined/summary_statistics"+title_tag+".pkl"
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

        for i in range(len(bd_type)):
            print(bd_type[i])
            best_performance_data.append([])
            performance_data.append([])
            best_transfer_data.append([])
            transfer_data.append([])
            resilience_data.append([])
            recovery_data.append([])
            for j in range(len(fitfuns)):
                print(fitfuns[j])
                BD_dir = get_bd_dir(fitfuns[j])
                # get all the data from the archive: no fault

                nofaultpath=BD_dir + "/" + bd_type[i] + "/FAULT_NONE/results"
                if bd_type[i]=="baseline":
                    best_nofaultperfs = get_baseline_performances(nofaultpath)
                    nofaultperfs=None
                    maxindsnofault=None
                else:
                    nofaultperfs, best_nofaultperfs, maxindsnofault = get_nofault_performances(nofaultpath,gener,runs)

                # join all the data from all the fault archives:
                performances = []
                best_performances = []
                recovery = []
                resilience = []
                transfer = []
                best_transfer = []
                for fault in range(len(faults)):
                    print("fault %d"%(fault))
                    for r, run in enumerate(runs):
                        faultpath = BD_dir + "/" + bd_type[i] + "/faultyrun" + str(run) + "_p" + str(fault) + "/results" + str(
                            run)
                        best_performances, performances, best_transfer, transfer, recovery,resilience = add_fault_performance(j, r, gener, nofaultperfs, best_nofaultperfs, maxindsnofault, faultpath,
                                              best_performances, performances, best_transfer, transfer, recovery,
                                              resilience, baseline=bd_type[i]=="baseline")
                            #otherwise transfer is undefined; we observe f=0 for some individuals in bordercoverage
                            # print(transfer.max())
                            # print(np.mean(transfer))

                if by_fitfun:
                    best_performance_data[i].append(best_performances)
                    performance_data[i].append(performances)
                    transfer_data[i].append(transfer)
                    best_transfer_data[i].append(best_transfer)
                    resilience_data[i].append(resilience)
                    recovery_data[i].append(recovery)
                else:
                    # normalise by maximal fitness (make different performance data comparable)
                    best_performance_data[i] = np.append(best_performance_data[i],best_performances/baseline_performances[fitfuns[j]])
                    performance_data[i] = np.append(performance_data[i],performances/baseline_performances[fitfuns[j]])
                    transfer_data[i] = np.append(transfer_data[i], transfer)
                    best_transfer_data[i] = np.append(best_transfer_data[i],best_transfer)
                    resilience_data[i] = np.append(resilience_data[i],resilience)
                    recovery_data[i] = np.append(recovery_data[i],recovery)

    from scipy.stats import mannwhitneyu
    all_data = (best_performance_data, performance_data, best_transfer_data, transfer_data,resilience_data)

    if by_fitfun:
        if not load_existing:
            pickle.dump(all_data,
                        open(loadfilename, "wb"))

        for i in range(len(best_performance_data)):
            for j in range(len(fitfuns)):
                best_performance_data[i][j]/=baseline_performances[fitfuns[j]]
        part_data = (best_performance_data, resilience_data)
        with open("results/fault/summary_table_fitfun"+title_tag,"w") as f:
            make_table(f,part_data,
                       rowlabels=fitfunlabels,
                       columnlabels=legend_labels,
                       conditionalcolumnlabels=[("bestperformance","float3"),("resilience","float3")],
                       transpose=False)
        with open("results/fault/summary_table_fitfun_median"+title_tag,"w") as f:
            make_table(f,part_data,
                       rowlabels=fitfunlabels,
                       columnlabels=legend_labels,
                       conditionalcolumnlabels=[("bestperformance","float3"),("resilience","float3")],
                       median=True,
                       transpose=False)
    else:
        if not load_existing:
            pickle.dump(all_data,
                        open(loadfilename, "wb"))
        part_data = [best_performance_data, performance_data, best_transfer_data, transfer_data, resilience_data]
        with open("results/summary_table"+title_tag,"w") as f:
            make_table(f,part_data,
                       rowlabels=legend_labels,
                       columnlabels=[],
                       conditionalcolumnlabels=[("bestperformance","float3"),("performance","float3"),("besttransfer","float3"),("transfer","float3"),("resilience","float3")])
        with open("results/summary_table_median"+title_tag, "w") as f:
            make_table(f, part_data,
                           rowlabels=legend_labels,
                           columnlabels=[],
                           conditionalcolumnlabels=[("bestperformance","float3"),("performance","float3"),("besttransfer","float3"),("transfer","float3"),("resilience","float3")],
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


def cliffs_delta(U,x,y):
    """

    meaning: proportion x>y minus proportion y>x
    |d|<0.147 "negligible", |d|<0.33 "small", |d|<0.474 "medium", otherwise "large"

    here calculate based on relation with the rank-sum test
    :param U: the result of the Wilcoxon rank-test/Mann-Withney U-test
    :return:
    """
    m=len(x)
    n=len(y)

    # delta =  2.0*U/float(m*n) - 1.0
    if len(x) > 2000 or len(y)>2000:   # avoid memory issues
        print("starting with lengths %d %d "%(m,n))
        print("digitising samples")
        xspace=np.linspace(x.min(),x.max(),500)
        yspace = np.linspace(x.min(), x.max(), 500)
        freq_x=np.histogram(x, bins=xspace)[0]
        freq_y=np.histogram(y, bins=yspace)[0]

        count=0
        for i in range(len(freq_x)):
            for j in range(len(freq_y)):
                num_combos=freq_x[i]*freq_y[j]
                xx=xspace[i]
                yy=yspace[j]
                if xx > yy:
                    count+=num_combos
                else:
                    count-=num_combos
        count/=float(m*n)
    else:
        z=np.array([xx - yy for xx in x for yy in y]) # consider all pairs of data
        count=float(sum(z>0) - sum(z<0))/float(m*n)
    # assert count==delta, "delta:%.3f  count:%.3f"%(delta,count)
    label = None
    magn=abs(count)
    if magn < 0.11:
        label="negligible"
    elif magn < 0.28:
        label="small"
    elif magn < 0.43:
        label="medium"
    else:
        label="large"
    return count, label
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
            BD_dir = get_bd_dir(fitfuns[j])
            # get all the data from the archive: no fault

            nofaultpath = BD_dir + "/" + bd_type[i] + "/FAULT_NONE/results"

            best_nofaultperfs = np.array([get_performance_data(nofaultpath + str(run), generation) for run in
                                          runs])
            max_performance = max(best_nofaultperfs)

            if max_performance>maximum[fitfuns[j]]:
                maximum[fitfuns[j]]=max_performance


    pickle.dump(maximum,open("data/fitfun/maximal_fitness.pkl","wb"))
if __name__ == "__main__":
    #test_NCD(num_agents=10, num_trials=10, num_ticks=100, num_features=8)




    # legend_labels.append("baseline")

    # set by_fitfun true
    significance_data(fitfuns, fitfunlabels, bd_type+["baseline"], runs, faults, generation, by_fitfun=False, load_existing=True,
                     title_tag="")
    #significance_data(fitfuns, fitfunlabels, bd_type+["baseline"], runs, faults, generation, by_fitfun=True, load_existing=True,
    #                 title_tag="")

    #significance_data(fitfuns, fitfunlabels, bd_type+["baseline"], runs, faults, generation, by_fitfun=True, load_existing=False,
    #                 title_tag="")
    #make_significance_table(fitfunlabels, legend_labels, qed_index=-2,table_type="performance")

    #make_significance_table(fitfunlabels, legend_labels, qed_index=-2, table_type="resilience")

    #gather_perturbation_results(datadir,generation,bd_type,fitfuns,faults,runs,history_type,perturbed=False)
    gather_perturbation_results(datadir, generation, bd_type, fitfuns, faults, runs, history_type, perturbed=True)
    #gather_perturbation_results(datadir, generation, bd_type, fitfuns, faults, runs, history_type, perturbed=False)