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
bd_type = ["history", "Gomes_sdbc_walls_and_robots_std", "cvt_rab_spirit", "environment_diversity"]  # legend label
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
                             history_type="sa",translation_type="handcrafted",centroids=[]):
    """
    use existing state/observation-action trajectory files and calculate pair-wise NCDs for all individuals within the sa;e run
    the average is returned
    :return:
    """
    ncds = []
    performances = []
    performance_comps = []
    dists = []
    categories = []

    for run in runs:
        history_comp, performance_comp, bd_comp = get_help_data(BD_DIRECTORY + "/FAULT_NONE/results"+str(run), generation,
                                                       history_type,translation_type)
        for fault in faults:
            history_file, performance, bd = \
                get_help_data(BD_DIRECTORY+"/faultyrun"+str(run)+"_p"+str(fault)+"/results"+str(run),generation,history_type,translation_type)
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
            if centroids:
                index,centr=transform_bd_cvtmapelites(bd, centroids)
                categories.append(index)



    return ncds, performances, performance_comps, dists, categories

def gather_unperturbed_diversity(BD_DIRECTORY,generation,faults, runs, get_NCD=True,
                             history_type="sa",translation_type="handcrafted",centroids=[]):
    """
    use existing state/observation-action trajectory files and calculate pair-wise NCDs for all individuals within the sa;e run
    the average is returned
    :return:
    """

    dists = []
    categories = []

    for run in runs:
        history_comp, performance_comp, bd_comp = get_help_data(BD_DIRECTORY + "/FAULT_NONE/results"+str(run), generation,
                                                       history_type,translation_type)
        for fault in faults:
            bd = get_help_data_unperturbed(BD_DIRECTORY,
                                           "/faultyrun"+str(run)+"_p"+str(fault)+"/results"+str(run),
                                           "/FAULT_NONE/results" + str(run),
                                           generation="30000",
                                           translation_type=translation_type)


            if translation_type=="spirit":
                dists.append(avg_variation_distance(bd_comp,bd,num_actions=16))
            else:
                dists.append(norm_Euclidian_dist(bd_comp,bd))
            if centroids:
                index,centr=transform_bd_cvtmapelites(bd, centroids)
                categories.append(index)



    return dists, categories



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

def estimation(max_density,condition,stats,x,xlabel, ylabel,
                   xlim, ylim,fig, ax,title,from_file=False):
    """
    :param data: list of observations
    :return:
    """

    #     algorithm : string
    #     The tree algorithm to use.  Valid options are
    #     ['kd_tree'|'ball_tree'|'auto'].  Default is 'auto'.
    # kernel : string
    #     The kernel to use.  Valid kernels are
    #     ['gaussian'|'tophat'|'epanechnikov'|'exponential'|'linear'|'cosine']
    #     Default is 'gau

    s_x = xlim[1] - xlim[0]
    s_y = ylim[1] - ylim[0]
    bandwidth = 0.05
    print("xrange"+str(xlim))
    print("yrange"+str(ylim))

    print("bandwidth="+str(bandwidth))

    num_y=int(np.ceil(s_y/bandwidth))
    num_x=int(np.ceil(s_x/bandwidth))
    print("num_cells" + str(num_x*num_y))
    print("maxprob=" + str(1 / float(num_x*num_y)))
    X = np.linspace(xlim[0], xlim[1], 100)
    Y = np.linspace(ylim[0], ylim[1], 100)
    if from_file:
        Z=pickle.load(open("data/combined/density/"+condition+"-"+xlabel+"-"+ylabel+".pkl","rb"))
    else:
        data = np.array([[xx, ss] for ss in stats for xx in x ])

        xy=np.array([[x,y] for y in Y for x in X] )



        X, Y = np.meshgrid(X, Y)

        kde = KernelDensity(bandwidth=bandwidth,
                      algorithm="kd_tree",
                      kernel ="gaussian",
                      metric ="euclidean",
                      atol = 0,
                      rtol = 0,
                      breadth_first = False,
                      leaf_size = 10,   # we will have a limited amount of data (+/- 1000 points)
                      metric_params = None)
        print("fitting")
        kde.fit(data)
        print("scoring")
        Z = np.exp(kde.score_samples(xy))
        Z = Z.reshape(X.shape)
        pickle.dump(Z,open("data/combined/density/"+condition+"-"+xlabel+"-"+ylabel+".pkl","wb"))
    #print("Z="+str(Z))
    # plot contours of the density
    assert Z.max()<=max_density, "%.2f vs %.2f "%(Z.max(),max_density)
    levels = np.linspace(0, max_density, 1001)
    print("plotting contour")
    CS=ax.contourf(X, Y, Z, levels=levels, cmap=plt.cm.Reds, vmin = 0, vmax = max_density)

    if title is not None:
        ax.set_title(title, fontsize=46)
    if ylim is not None:
        ax.set_ylim(ylim)
    if xlim is not None:
        ax.set_xlim(xlim)
    # if xlabel is not None:
    #     ax.set_xlabel(xlabel, fontsize=36)
    # if ylabel is not None:
    #     ax.set_ylabel(ylabel, fontsize=36)
    ax.tick_params(axis='both', which='major', labelsize=24)
    ax.tick_params(axis='both', which='minor', labelsize=24)

    return CS



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
    for bd in bd_type:
        for fitfun in fitfuns:
            title = fitfun + "range0.11"
            prefix = datadir + "/" + title + "/" + bd


            if perturbed:
                ncds, performances, nofaultperfs, euclids, categories= gather_perturbation_data(prefix, generation, faults,
                                                                                    runs=runs, history_type=history_type,
                                                                                    translation_type="sdbc",centroids=centroids_sdbc,get_NCD=False)
                _, _, _, maxvars,_= gather_perturbation_data(prefix, generation, faults,
                                                                   runs=runs, history_type=history_type,
                                                                   translation_type="spirit",get_NCD=False)

                _, _, _ , _, categories_handcrafted = gather_perturbation_data(prefix, generation, faults,
                                                                                    runs=runs, history_type=history_type,
                                                                                    translation_type="handcrafted",centroids=centroids,get_NCD=False)
                dp_file,ncd_file, euclid_file, maxvars_file, category_file, category_h_file = filenames(fitfun,bd,history_type)
                pickle.dump((performances,nofaultperfs), open(dp_file, "wb"))
                pickle.dump(ncds, open(ncd_file, "wb"))
                pickle.dump(euclids, open(euclid_file, "wb"))
                pickle.dump(maxvars, open(maxvars_file, "wb"))
                pickle.dump(categories, open(category_file, "wb"))
                pickle.dump(categories_handcrafted, open(category_h_file, "wb"))
            else:

                _,categories_handcrafted = gather_unperturbed_diversity(prefix,generation, faults, runs,
                                             get_NCD=False,
                                             history_type=history_type,
                                             translation_type="handcrafted", centroids=centroids
                                             )
                euclids,categories = gather_unperturbed_diversity(prefix,generation, faults, runs,
                                             get_NCD=False,
                                             history_type=history_type,
                                             translation_type="sdbc", centroids=centroids_sdbc
                                             )

                maxvars, _ = gather_unperturbed_diversity(prefix,generation, faults, runs,
                                             get_NCD=False,
                                             history_type=history_type,
                                             translation_type="spirit", centroids=[]
                                             )

                euclid_file, maxvar_file, category_file, categoryh_file = unperturbed_filenames(fitfun,bd,history_type)
                pickle.dump(euclids, open(euclid_file, "wb"))
                pickle.dump(maxvars, open(maxvar_file, "wb"))
                pickle.dump(categories, open(category_file, "wb"))
                pickle.dump(categories_handcrafted, open(categoryh_file, "wb"))
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
          prefix +  "_categories.pkl", prefix +  "handcrafted_categories.pkl"
def unperturbed_filenames(fitfun,bd,history_type):
    #dp_file, ncd_file, euclid_file, ent_file, category_file
    prefix= "data/fitfun/"+fitfun+"/"+ bd + history_type
    return prefix +"_euclidsunperturbed.pkl",prefix +"_maxvarsunperturbed.pkl", \
          prefix +  "_categoriesunperturbed.pkl", prefix +  "handcrafted_categoriesunperturbed.pkl"
def plot_by_fitfun(leg_labels,titles,plot_NCD=False):
    fig1, axs1 = plt.subplots(1, 5, figsize=(50, 10))
    fig2, axs2 = plt.subplots(1, 5, figsize=(50, 10))
    fig3, axs3 = plt.subplots(1, 5, figsize=(50, 10))
    fig4, axs4 = plt.subplots(1, 5, figsize=(50, 10))
    fig5, axs5 = plt.subplots(1, 5, figsize=(50, 10))
    fig6, axs6 = plt.subplots(1, 5, figsize=(50, 10))
    fig7, axs7 = plt.subplots(1, 5, figsize=(50, 10))
    fig8, axs8 = plt.subplots(1, 5, figsize=(50, 10))
    fig9, axs9 = plt.subplots(1, 5, figsize=(50, 10))
    fig10, axs10 = plt.subplots(1, 5, figsize=(50, 10))
    xlim2_dict={"Aggregation":[0.60,1.0],"Dispersion":[0,0.25],"Flocking":[0,0.20],"DecayCoverage":[0.60,1],"DecayBorderCoverage":[0.60,1.0]}
    for i, fitfun in enumerate(fitfuns):
        stats = []
        stats2=[]
        stats3=[]
        stats4=[]
        stats5=[]
        x2=[]
        x = []
        for bd in bd_type:
            dp_file, ncd_file, euclid_file, ent_file, category_file, category_h_file = filenames(fitfun, bd, history_type)
            performances , nofaultperfs = pickle.load(open(dp_file, "rb"))
            dps = np.array(performances) - np.array(nofaultperfs)
            ncds = pickle.load(open(ncd_file, "rb"))
            stats.append(ncds)
            euclids = pickle.load(open(euclid_file,"rb"))
            ents = pickle.load(open(ent_file,"rb"))
            categories=pickle.load(open(category_file,"rb"))
            categories_h = pickle.load(open(category_h_file, "rb"))
            stats2.append(euclids)
            stats3.append(ents)
            stats4.append(categories)
            stats5.append(categories_h)
            x2.append(np.array(performances))
            x.append(np.array(dps))
        #print(stats)
        #print(x)
        xlim2=xlim2_dict[fitfun]
        xlim=[-xlim2[1]/5.,+xlim2[1]/20.]
        if plot_NCD:
            createPlot(stats, x, colors, markers, xlabel="performance change", ylabel="$NCD$",
                       xlim=xlim, ylim=[0, 1], save_filename="results/fault/density/NCD_DELTAP.pdf",
                       legend_labels=leg_labels, scatter=True, force=True,
                       ax=axs1[i],title=titles[i])

            createPlot(stats, x2, colors, markers, xlabel="performance", ylabel="$NCD$",
                       xlim=xlim2, ylim=[0, 1], save_filename="results/fault/density/NCD_P.pdf",
                       legend_labels=leg_labels, scatter=True, force=True,
                       ax=axs2[i],title=titles[i])


        createPlot(stats2, x, colors, markers, xlabel="performance change", ylabel="Euclidian distance",
                   xlim=xlim, ylim=[0, 1], save_filename="results/fault/density/Euclid_DELTAP.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs3[i],title=titles[i])

        createPlot(stats2, x2, colors, markers, xlabel="performance", ylabel="Euclidian distance",
                   xlim=xlim2, ylim=[0, 1], save_filename="results/fault/density/Euclid_P.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs4[i],title=titles[i])

        createPlot(stats3, x, colors, markers, xlabel="performance change", ylabel="maximum variation distance",
                   xlim=xlim,ylim=[0,1.0], save_filename="results/fault/density/MAXVAR_DELTAP.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs5[i],title=titles[i])

        createPlot(stats3, x2, colors, markers, xlabel="performance", ylabel="maximum variation distance",
                   xlim=xlim2, ylim=[0, 1.0], save_filename="results/fault/density/MAXVAR_P.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs6[i],title=titles[i])

        createPlot(stats4, x, colors, markers, xlabel="performance change", ylabel="category",
                   xlim=xlim, ylim=[0, 10], save_filename="results/fault/density/category_DELTAP.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs7[i],title=titles[i])

        createPlot(stats4, x2, colors, markers, xlabel="performance", ylabel="category",
                   xlim=xlim2, ylim=[0, 10], save_filename="results/fault/density/category_P_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs8[i],title=titles[i])


        createPlot(stats5, x, colors, markers, xlabel="performance change", ylabel="category",
                   xlim=xlim, ylim=[0, 10], save_filename="results/fault/density/category_DELTAP.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs9[i],title=titles[i])

        createPlot(stats5, x2, colors, markers, xlabel="performance", ylabel="category",
                   xlim=xlim2, ylim=[0, 10], save_filename="results/fault/density/category_P_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs10[i],title=titles[i])
    finish_fig(fig1, "results/fault/density/NCD_DELTAP.pdf")
    finish_fig(fig2, "results/fault/density/NCD_P.pdf")
    finish_fig(fig3, "results/fault/density/Euclid_DELTAP.pdf")
    finish_fig(fig4, "results/fault/density/Euclid_P.pdf")
    finish_fig(fig5, "results/fault/density/MAXVAR_DELTAP.pdf")
    finish_fig(fig6, "results/fault/density/MAXVAR_P.pdf")
    finish_fig(fig7, "results/fault/density/Category_DELTAP.pdf")
    finish_fig(fig8, "results/fault/density/Category_P.pdf")
    finish_fig(fig9, "results/fault/density/CategoryH_DELTAP.pdf")
    finish_fig(fig10, "results/fault/density/CategoryH_P.pdf")

def plot_by_descriptor(leg_labels,titles,xlim):
    fig1, axs1 = plt.subplots(1, 4, figsize=(40, 10))
    fig2, axs2 = plt.subplots(1, 4, figsize=(40, 10))
    fig3, axs3 = plt.subplots(1, 4, figsize=(40, 10))
    fig4, axs4 = plt.subplots(1, 4, figsize=(40, 10))
    fig5, axs5 = plt.subplots(1, 4, figsize=(40, 10))
    fig6, axs6 = plt.subplots(1, 4, figsize=(40, 10))
    fig7, axs7 = plt.subplots(1, 4, figsize=(40, 10))
    fig8, axs8 = plt.subplots(1, 4, figsize=(40, 10))
    fig9, axs9 = plt.subplots(1, 4, figsize=(40, 10))
    fig10, axs10 = plt.subplots(1, 4, figsize=(40, 10))

    for i, bd in enumerate(bd_type):

        stats = []
        stats2=[]
        stats3=[]
        stats4=[]
        stats5=[]
        x2=[]
        x = []
        for fitfun in fitfuns:
            dp_file, ncd_file, euclid_file, ent_file, category_file, category_h_file = filenames(fitfun, bd, history_type)
            performances , nofaultperfs = pickle.load(open(dp_file, "rb"))
            dps = np.array(performances) - np.array(nofaultperfs)
            ncds = pickle.load(open(ncd_file, "rb"))
            stats.append(ncds)
            euclids = pickle.load(open(euclid_file,"rb"))
            ents = pickle.load(open(ent_file,"rb"))
            categories = pickle.load(open(category_file, "rb"))
            categories_h = pickle.load(open(category_h_file, "rb"))
            stats2.append(euclids)
            stats3.append(ents)
            x2.append(np.array(performances))
            x.append(np.array(dps))
            stats4.append(categories)
            stats5.append(categories_h)
        #print(stats)
        #print(x)

        createPlot(stats, x, colors, markers, xlabel="$\Delta P$", ylabel="$NCD$",
                   xlim=xlim, ylim=[0, 1], save_filename="results/fault/density/NCD_DELTAP_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs1[i],title=titles[i])

        createPlot(stats, x2, colors, markers, xlabel="$P$", ylabel="$NCD$",
                   xlim=[0,1], ylim=[0, 1], save_filename="results/fault/density/NCD_P_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs2[i],title=titles[i])


        createPlot(stats2, x, colors, markers, xlabel="$\Delta P$", ylabel="Euclidian distance",
                   xlim=xlim, ylim=[0, 1], save_filename="results/fault/density/Euclid_DELTAP_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs3[i],title=titles[i])

        createPlot(stats2, x2, colors, markers, xlabel="$P$", ylabel="Euclidian distance",
                   xlim=[0,1], ylim=[0, 1], save_filename="results/fault/density/Euclid_P_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs4[i],title=titles[i])

        createPlot(stats3, x, colors, markers, xlabel="$\Delta P$", ylabel="maximum variation distance",
                   xlim=xlim, ylim=[0, 1.0], save_filename="results/fault/density/MAXVAR_DELTAP_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs5[i],title=titles[i])

        createPlot(stats3, x2, colors, markers, xlabel="$P$", ylabel="maximum variation distance",
                   xlim=[0,1], ylim=[0, 1.0], save_filename="results/fault/density/MAXVAR_P_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs6[i],title=titles[i])

        createPlot(stats4, x, colors, markers, xlabel="$\Delta P$", ylabel="category",
                   xlim=xlim, ylim=[0, 10], save_filename="results/fault/density/category_DELTAP_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs7[i],title=titles[i])

        createPlot(stats4, x2, colors, markers, xlabel="$P$", ylabel="category",
                   xlim=[0,1], ylim=[0, 10], save_filename="results/fault/density/category_P_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs8[i],title=titles[i])

        createPlot(stats5, x, colors, markers, xlabel="performance change", ylabel="category",
                   xlim=xlim, ylim=[0, 10], save_filename="results/fault/density/category_DELTAP.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs9[i],title=titles[i])

        createPlot(stats5, x2, colors, markers, xlabel="performance", ylabel="category",
                   xlim=[0,1], ylim=[0, 10], save_filename="results/fault/density/category_P_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs10[i],title=titles[i])


    finish_fig(fig1, "results/fault/density/NCD_DELTAP_desc.pdf")
    finish_fig(fig2, "results/fault/density/NCD_P_desc.pdf")
    finish_fig(fig3, "results/fault/density/Euclid_DELTAP_desc.pdf")
    finish_fig(fig4, "results/fault/density/Euclid_P_desc.pdf")
    finish_fig(fig5, "results/fault/density/MAXVAR_DELTAP_desc.pdf")
    finish_fig(fig6, "results/fault/density/MAXVAR_P_desc.pdf")
    finish_fig(fig7, "results/fault/density/Category_DELTAP_desc.pdf")
    finish_fig(fig8, "results/fault/density/Category_P_desc.pdf")
    finish_fig(fig9, "results/fault/density/CategoryH_DELTAP_desc.pdf")
    finish_fig(fig10, "results/fault/density/CategoryH_P_desc.pdf")


def plot_proportional_byfitfun(leg_labels,titles,plot_NCD=False):
    fig1, axs1 = plt.subplots(1, 5, figsize=(50, 10))
    fig2, axs2 = plt.subplots(1, 5, figsize=(50, 10))
    fig3, axs3 = plt.subplots(1, 5, figsize=(50, 10))
    fig4, axs4 = plt.subplots(1, 5, figsize=(50, 10))
    fig5, axs5 = plt.subplots(1, 5, figsize=(50, 10))
    fig6, axs6 = plt.subplots(1, 5, figsize=(50, 10))
    fig7, axs7 = plt.subplots(1, 5, figsize=(50, 10))
    fig8, axs8 = plt.subplots(1, 5, figsize=(50, 10))
    fig9, axs9 = plt.subplots(1, 4, figsize=(40, 10))
    fig10, axs10 = plt.subplots(1, 4, figsize=(40, 10))
    xlim2_dict={"Aggregation":[0.60,1.0],"Dispersion":[0,0.25],"Flocking":[0,0.20],"DecayCoverage":[0.60,1],"DecayBorderCoverage":[0.60,1.0]}
    for i, fitfun in enumerate(fitfuns):
        stats = []
        stats2=[]
        stats3=[]
        stats4=[]
        stats5=[]
        x2=[]
        x = []
        for bd in bd_type:
            dp_file, ncd_file, euclid_file, ent_file, category_file, category_h_file = filenames(fitfun, bd, history_type)
            performances , nofaultperfs = pickle.load(open(dp_file, "rb"))
            dps = np.array(performances) - np.array(nofaultperfs)
            ncds = pickle.load(open(ncd_file, "rb"))
            stats.append(ncds)
            euclids = pickle.load(open(euclid_file,"rb"))
            ents = pickle.load(open(ent_file,"rb"))
            categories=pickle.load(open(category_file,"rb"))
            categories_h = pickle.load(open(category_h_file,"rb"))
            stats2.append(euclids)
            stats3.append(ents)
            stats4.append(categories)
            stats5.append(categories_h+1)
            x2.append(np.array(performances))
            x.append(np.array(dps)/np.array(nofaultperfs))
        #print(stats)
        #print(x)
        xlim2=xlim2_dict[fitfun]
        xlim=[-.25,.05]
        if plot_NCD:
            createPlot(stats, x, colors, markers, xlabel="resilience", ylabel="$NCD$",
                       xlim=xlim, ylim=[0, 1], save_filename="results/fault/density/NCD_DELTAP.pdf",
                       legend_labels=leg_labels, scatter=True, force=True,
                       ax=axs1[i],title=titles[i])

            createPlot(stats, x2, colors, markers, xlabel="performance", ylabel="$NCD$",
                       xlim=xlim2, ylim=[0, 1], save_filename="results/fault/density/NCD_P.pdf",
                       legend_labels=leg_labels, scatter=True, force=True,
                       ax=axs2[i],title=titles[i])


        createPlot(stats2, x, colors, markers, xlabel="resilience", ylabel="Euclidian distance",
                   xlim=xlim, ylim=[0, 1], save_filename="results/fault/density/Euclid_DELTAP.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs3[i],title=titles[i])

        createPlot(stats2, x2, colors, markers, xlabel="performance", ylabel="Euclidian distance",
                   xlim=xlim2, ylim=[0, 1], save_filename="results/fault/density/Euclid_P.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs4[i],title=titles[i])

        createPlot(stats3, x, colors, markers, xlabel="resilience", ylabel="maximum variation distance",
                   xlim=xlim,ylim=[0,1.0], save_filename="results/fault/density/MAXVAR_DELTAP.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs5[i],title=titles[i])

        createPlot(stats3, x2, colors, markers, xlabel="performance", ylabel="maximum variation distance",
                   xlim=xlim2, ylim=[0, 1.0], save_filename="results/fault/density/MAXVAR_P.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs6[i],title=titles[i])

        createPlot(stats4, x, colors, markers, xlabel="resilience", ylabel="category",
                   xlim=xlim, ylim=[0, 11], save_filename="results/fault/density/category_DELTAP.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs7[i],title=titles[i])

        createPlot(stats4, x2, colors, markers, xlabel="performance", ylabel="category",
                   xlim=xlim2, ylim=[0, 11], save_filename="results/fault/density/category_P_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs8[i],title=titles[i])
        createPlot(stats5, x, colors, markers, xlabel="performance change", ylabel="category",
                   xlim=xlim, ylim=[0, 11], save_filename="results/fault/density/category_DELTAP.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs9[i],title=titles[i])

        createPlot(stats5, x2, colors, markers, xlabel="performance", ylabel="category",
                   xlim=[0,1], ylim=[0, 11], save_filename="results/fault/density/category_P_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs10[i],title=titles[i])
    finish_fig(fig1, "results/fault/density/proportional_NCD_DELTAP.pdf")
    finish_fig(fig2, "results/fault/density/proportional_NCD_P.pdf")
    finish_fig(fig3, "results/fault/density/proportional_Euclid_DELTAP.pdf")
    finish_fig(fig4, "results/fault/density/proportional_Euclid_P.pdf")
    finish_fig(fig5, "results/fault/density/proportional_MAXVAR_DELTAP.pdf")
    # finish_fig(fig6, "results/fault/density/proportional_MAXVAR_P.pdf")
    finish_fig(fig7, "results/fault/density/proportional_Category_DELTAP.pdf")
    # finish_fig(fig8, "results/fault/density/proportional_Category_P.pdf")
    finish_fig(fig9, "results/fault/density/proportional_CategoryH_DELTAP.pdf")
    # finish_fig(fig10, "results/fault/density/proportional_CategoryH_P.pdf")


def convert_resilience_data_for_check(resilience_data):

    new_array=np.zeros(1000)
    j=0
    for fitfun in range(5):
        fitfun_offset = fitfun*200
        for run in range(5):
            for fault in range(0,40):
                fault_offset = fault*5
                i = fitfun_offset + fault_offset + run

                new_array[j] = resilience_data[i]
                j+=1
    return new_array


def plot_density_bydescriptor(titles,plot_NCD=False):
    fig1, axs1 = plt.subplots(1, 4, figsize=(40, 10))
    fig2, axs2 = plt.subplots(1, 4, figsize=(40, 10))
    fig3, axs3 = plt.subplots(1, 4, figsize=(40, 10))
    fig4, axs4 = plt.subplots(1, 4, figsize=(40, 10))
    fig5, axs5 = plt.subplots(1, 4, figsize=(40, 10))
    fig6, axs6 = plt.subplots(1, 4, figsize=(40, 10))
    fig7, axs7 = plt.subplots(1, 4, figsize=(40, 10))
    fig8, axs8 = plt.subplots(1, 4, figsize=(40, 10))
    fig9, axs9 = plt.subplots(1, 4, figsize=(40, 10))
    fig10, axs10 = plt.subplots(1, 4, figsize=(40, 10))
    best_performance_data, performance_data, best_transfer_data, transfer_data, resilience_data = pickle.load(
        open("data/combined/summary_statistics.pkl", "rb"))
    for i, bd in enumerate(bd_type):
        print(bd)
        stats = []
        stats2 = []
        stats3 = []
        stats4 = []
        stats5 = []
        x2 = []
        x = []
        for fitfun in fitfuns:
            print(fitfun)
            dp_file, ncd_file, euclid_file, ent_file, category_file, category_h_file = filenames(fitfun, bd, history_type)
            performances, nofaultperfs = pickle.load(open(dp_file, "rb"))
            dps = np.array(performances) - np.array(nofaultperfs)
            ncds = pickle.load(open(ncd_file, "rb"))
            stats.append(ncds)
            euclids = pickle.load(open(euclid_file, "rb"))
            ents = pickle.load(open(ent_file, "rb"))
            categories = pickle.load(open(category_file, "rb"))
            categories_h = pickle.load(open(category_h_file, "rb"))
            stats2= np.append(stats2,euclids)
            stats3= np.append(stats3,ents)
            x2=np.append(x2,performances)
            x=np.append(x,np.array(dps) / np.array(nofaultperfs))
            stats4=np.append(stats4,categories)
            stats5=np.append(stats5,categories_h)

        newarray=convert_resilience_data_for_check(resilience_data[i])
        assert np.allclose(newarray,x)
        xlim = [-0.30,0.0]
        xlim2 = [0,1]
        ylim=[0,1.0]
        if plot_NCD:
            cs1=estimation(stats,x,xlabel="resilience",
                           ylabel="$NCD$", xlim=xlim, ylim=ylim, fig=fig1, ax=axs1[i],title=titles[i])
            # cs2=estimation(stats,x2,xlabel="performance",
            #                ylabel="$NCD$", xlim=xlim2, ylim=ylim, fig=fig2, ax=axs2[i],title=titles[i])
        cs3=estimation(40,bd,stats2, resilience_data[i],  xlabel="resilience",
                        ylabel="Euclidian distance", xlim=xlim, ylim=ylim, fig=fig3, ax=axs3[i], title=titles[i],from_file=False)
        # cs4=estimation(stats2, x2,  xlabel="performance",
        #                    ylabel="Euclidian distance", xlim=xlim2, ylim=ylim, fig=fig4, ax=axs4[i], title=titles[i])

        #cs5=estimation(20, bd,stats3, x,  xlabel="resilience",
        #           ylabel="maximum variation distance", xlim=xlim, ylim=ylim, fig=fig5, ax=axs5[i], title=titles[i],from_file=False)
        # # cs6=estimation(stats3, x2,  xlabel="performance",
        # #                    ylabel="maximum variation distance", xlim=xlim2, ylim=ylim, fig=fig6, ax=axs6[i], title=titles[i])
        #
        #
        #cs7=estimation(stats4+1, x,  xlabel="resilience",
        #                 ylabel="category", xlim=xlim, ylim=[0,11], fig=fig7, ax=axs7[i], title=titles[i])
        # # cs8=estimation(stats4+1, x2,  xlabel="performance",
        # #                    ylabel="category", xlim=xlim2, ylim=[0,9.5], fig=fig8, ax=axs8[i], title=titles[i])
        #
        #cs9=estimation(0.08,bd, stats5+1, x,  xlabel="resilience",
        #              ylabel="category", xlim=xlim, ylim=[0,11], fig=fig9, ax=axs9[i], title=titles[i],from_file=True)
        # cs10=estimation(stats5+1, x2,  xlabel="performance",
        #                    ylabel="category", xlim=xlim2, ylim=[0,9.5], fig=fig10, ax=axs10[i], title=titles[i])
    if plot_NCD:
        finish_fig(fig1, "results/fault/density/NCD_DELTAP_desc.pdf",cs1)
        # finish_fig(fig2, "results/fault/density/NCD_P_desc.pdf",cs2)
    fig3.text(0.5, 0.01, 'resilience', ha='center', fontsize=36)
    fig3.text(0.09, 0.5, 'diversity', va='center', rotation='vertical', fontsize=36)
    finish_fig(fig3, "results/fault/density/CHECKEuclid_DELTAP_desc.pdf",(cs3,[r"$0\%$","$0.04\%$","$0.08\%$"]))

    # finish_fig(fig4, "results/fault/density/Euclid_P_desc.pdf",cs4)
    #fig5.text(0.5, 0.01, 'resilience', ha='center',fontsize=36)
    #fig5.text(0.09, 0.5, 'diversity', va='center', rotation='vertical',fontsize=36)
    #finish_fig(fig5, "results/fault/density/MAXVAR_DELTAP_desc.pdf",(cs5,[r"$0\%$","$0.04\%$","$0.08\%$"]))
    # finish_fig(fig6, "results/fault/density/MAXVAR_P_desc.pdf",cs6)
    #finish_fig(fig7, "results/fault/density/Category_DELTAP_desc.pdf",cs7)
    # finish_fig(fig8, "results/fault/density/CategoryH_P_desc.pdf",cs8)
    # fig9.text(0.5, 0.01, 'resilience', ha='center', fontsize=36)
    # fig9.text(0.09, 0.5, 'category', va='center', rotation='vertical', fontsize=36)
    # finish_fig(fig9, "results/fault/density/CategoryH_DELTAP_desc.pdf",cs9)
    # finish_fig(fig10, "results/fault/density/CategoryH_P_desc.pdf",cs10)

def plot_density_bydescriptor_unperturbed(titles,plot_NCD=False):
    fig1, axs1 = plt.subplots(1, 4, figsize=(40, 10))
    fig2, axs2 = plt.subplots(1, 4, figsize=(40, 10))
    fig3, axs3 = plt.subplots(1, 4, figsize=(40, 10))
    fig4, axs4 = plt.subplots(1, 4, figsize=(40, 10))
    fig5, axs5 = plt.subplots(1, 4, figsize=(40, 10))
    fig6, axs6 = plt.subplots(1, 4, figsize=(40, 10))
    fig7, axs7 = plt.subplots(1, 4, figsize=(40, 10))
    fig8, axs8 = plt.subplots(1, 4, figsize=(40, 10))
    fig9, axs9 = plt.subplots(1, 4, figsize=(40, 10))
    fig10, axs10 = plt.subplots(1, 4, figsize=(40, 10))



    for i, bd in enumerate(bd_type):
        print(bd)
        stats = []
        stats2 = []
        stats3 = []
        stats4 = []
        stats5 = []
        x2 = []
        x = []
        for fitfun in fitfuns:
            print(fitfun)
            dp_file, _, _, _, _, _ = filenames(fitfun, bd,history_type)
            euclid_file, ent_file, category_file, category_h_file = unperturbed_filenames(fitfun, bd, history_type)
            performances, nofaultperfs = pickle.load(open(dp_file, "rb"))
            dps = np.array(performances) - np.array(nofaultperfs)
            euclids = pickle.load(open(euclid_file, "rb"))
            ents = pickle.load(open(ent_file, "rb"))
            categories = pickle.load(open(category_file, "rb"))
            categories_h = pickle.load(open(category_h_file, "rb"))
            stats2= np.append(stats2,euclids)
            stats3= np.append(stats3,ents)
            x2=np.append(x2,performances)
            x=np.append(x,np.array(dps) / np.array(nofaultperfs))
            stats4=np.append(stats4,categories)
            stats5=np.append(stats5,categories_h)

        #print(stats)
        #print(x)
        xlim = [-0.30,0.0]
        xlim2 = [0,1]
        ylim=[0,1.0]
        if plot_NCD:
            cs1=estimation(stats,x,xlabel="resilience",
                           ylabel="$NCD$", xlim=xlim, ylim=ylim, fig=fig1, ax=axs1[i],title=titles[i])
            # cs2=estimation(stats,x2,xlabel="performance",
            #                ylabel="$NCD$", xlim=xlim2, ylim=ylim, fig=fig2, ax=axs2[i],title=titles[i])
        cs3=estimation(20,bd,stats2, x,  xlabel="resilience",
                        ylabel="Euclidian distance", xlim=xlim, ylim=ylim, fig=fig3, ax=axs3[i], title=titles[i],from_file=False)
        # cs4=estimation(stats2, x2,  xlabel="performance",
        #                    ylabel="Euclidian distance", xlim=xlim2, ylim=ylim, fig=fig4, ax=axs4[i], title=titles[i])

        cs5=estimation(20, bd,stats3, x,  xlabel="resilience",
                   ylabel="maximum variation distance", xlim=xlim, ylim=ylim, fig=fig5, ax=axs5[i], title=titles[i],from_file=False)
        # # cs6=estimation(stats3, x2,  xlabel="performance",
        # #                    ylabel="maximum variation distance", xlim=xlim2, ylim=ylim, fig=fig6, ax=axs6[i], title=titles[i])
        #
        #
        #cs7=estimation(stats4+1, x,  xlabel="resilience",
        #                 ylabel="category", xlim=xlim, ylim=[0,11], fig=fig7, ax=axs7[i], title=titles[i])
        # # cs8=estimation(stats4+1, x2,  xlabel="performance",
        # #                    ylabel="category", xlim=xlim2, ylim=[0,9.5], fig=fig8, ax=axs8[i], title=titles[i])
        #
        #cs9=estimation(0.08,bd, stats5+1, x,  xlabel="resilience",
        #              ylabel="category", xlim=xlim, ylim=[0,11], fig=fig9, ax=axs9[i], title=titles[i],from_file=True)
        # cs10=estimation(stats5+1, x2,  xlabel="performance",
        #                    ylabel="category", xlim=xlim2, ylim=[0,9.5], fig=fig10, ax=axs10[i], title=titles[i])
    if plot_NCD:
        finish_fig(fig1, "results/fault/density/NCD_DELTAP_desc.pdf",cs1)
        # finish_fig(fig2, "results/fault/density/NCD_P_desc.pdf",cs2)
    fig3.text(0.5, 0.01, 'resilience', ha='center', fontsize=36)
    fig3.text(0.09, 0.5, 'diversity', va='center', rotation='vertical', fontsize=36)
    finish_fig(fig3, "results/fault/density/Euclid_DELTAPunperturbed_desc.pdf",(cs3,[r"$0\%$","$0.04\%$","$0.08\%$"]))

    # finish_fig(fig4, "results/fault/density/Euclid_P_desc.pdf",cs4)
    fig5.text(0.5, 0.01, 'resilience', ha='center',fontsize=36)
    fig5.text(0.09, 0.5, 'diversity', va='center', rotation='vertical',fontsize=36)
    finish_fig(fig5, "results/fault/density/MAXVAR_DELTAPunperturbed_desc.pdf",(cs5,[r"$0\%$","$0.04\%$","$0.08\%$"]))
    # finish_fig(fig6, "results/fault/density/MAXVAR_P_desc.pdf",cs6)
    #finish_fig(fig7, "results/fault/density/Category_DELTAP_desc.pdf",cs7)
    # finish_fig(fig8, "results/fault/density/CategoryH_P_desc.pdf",cs8)
    # fig9.text(0.5, 0.01, 'resilience', ha='center', fontsize=36)
    # fig9.text(0.09, 0.5, 'category', va='center', rotation='vertical', fontsize=36)
    # finish_fig(fig9, "results/fault/density/CategoryH_DELTAP_desc.pdf",cs9)
    # finish_fig(fig10, "results/fault/density/CategoryH_P_desc.pdf",cs10)

def perturbed_vs_unperturbed_best(fitfuns,fitfunlabels,bd_type,bd_labels,save_file,ylim):

    data=[]
    pickle.load("data/fitfun/summary_statustics.pkl")
    for i in range(len(fitfuns)):
        data.append([])
        for j in range(len(bd_type)):
            # get the best performances, fault vs no_fault

            data[i].append([nofaultperfs,performances])

    #make_boxplot_matrix(data, fitfuns, bd_labels, save_file, xlabs=["no perturb.","perturb."], ylab="performance",ylim=ylim)
    make_boxplot_pairswithin(data, fitfunlabels, bd_labels, save_file, xlabs=bd_labels, ylab="performance",
                        ylim=ylim)
def perturbed_vs_unperturbed_archive(fitfuns,fitfunlabels,bd_type,runs,faults,gener,bd_labels,save_file,ylim):

    data=[]
    for i in range(len(fitfuns)):
        BD_dir = get_bd_dir(fitfuns[i])
        data.append([])
        for j in range(len(bd_type)):
            # get all the data from the archive: no fault
            nofaultperfs=np.array(list(get_combined_archive(BD_dir+"/"+bd_type[j]+"/FAULT_NONE",runs,"analysis"+str(gener)+"_handcrafted.dat").values())).flatten()
            # join all the data from all the fault archives:
            performances=[]
            for fault in range(len(faults)):
                for run in runs:
                    temp=np.array(list(get_bin_performances_uniquearchive(BD_dir+"/"+bd_type[j]+"/run"+str(run)+"_p"+str(fault)+"/results"+str(run)+"/analysis"+str(gener)+"_handcrafted.dat").values())).flatten()
                    performances=np.append(performances,temp)

            data[i].append([nofaultperfs,performances])

    #make_boxplot_matrix(data, fitfuns, bd_labels, save_file, xlabs=["no perturb.","perturb."], ylab="performance",ylim=ylim)
    make_boxplot_pairswithin(data, fitfunlabels, bd_labels, save_file, xlabs=bd_labels, ylab="performance",
                        ylim=ylim)
def convert_to_dict():
    ###
    best_performance_data, performance_data, best_transfer_data, transfer_data, resilience_data = pickle.load(
        open("data/combined/summary_statistics.pkl", "rb"))
    d = {"best performance": best_performance_data,
         "performance":performance_data, "best transfer":best_transfer_data,
         "transfer":transfer_data, "resilience":resilience_data}
    pickle.dump(d,open("data/combined/summary_statistics_dict.pkl","wb"))

    #######
    best_performance_data, performance_data, best_transfer_data, transfer_data, resilience_data = pickle.load(
        open("data/fitfun/summary_statistics_fitfun.pkl", "rb"))
    d = {"best_performance": best_performance_data,
         "performance":performance_data, "best_transfer":best_transfer_data,
         "transfer":transfer_data, "resilience":resilience_data}
    pickle.dump(d,open("data/fitfun/summary_statistics_fitfun_dict.pkl","wb"))


def summarystatistic_boxplots(labels):
    # resilience_data is list of numpy arrays, implying bd is row, but bds to be colums
    convert_to_dict()
    data = pickle.load(
        open("data/combined/summary_statistics_dict.pkl", "rb"))


    make_boxplots_from_dict(data,save_filename="boxplot_",
                        xlabs=labels,ylim=[-1.0,1.0])

# def transfer_boxplot(transfer_data):
#
#
# def besttransfer_boxplot(best_transfer_data):





def get_baseline_performances(nofaultpath):
        nofaultfilenames = [nofaultpath + str(run) + "/fitness" for run in runs]
        nofaultperfs = [get_baseline_fitness(f) for f in
                        nofaultfilenames]

        return nofaultperfs

def add_fault_performance(j, r, gener, nofaultperfs,best_nofaultperfs,maxindsnofault,faultpath,
                          best_performances,performances,best_transfer,transfer,recovery,resilience, baseline=False):
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
            transfer = np.append(transfer, [(temp[k] - nofaultperfs[r][k]) / baseline_performances[fitfuns[j]]])

        best_transfer = np.append(best_transfer,
                                  (temp[maxindsnofault[r]] - best_nofaultperfs[r]) / best_nofaultperfs[r])

    best_performances = np.append(best_performances, best_performance)

    recovery = np.append(recovery, [(best_performance - best_nofaultperfs[r]) / baseline_performances[
        fitfuns[j]]])  # best performance vs best nofaultperf
    resilience = np.append(resilience, (best_performance - best_nofaultperfs[r]) / best_nofaultperfs[r])

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
            make_table(f,all_data,
                       rowlabels=legend_labels,
                       columnlabels=[],
                       conditionalcolumnlabels=[("bestperformance","float3"),("performance","float3"),("besttransfer","float3"),("transfer","float3"),("resilience","float3")])
        with open("results/summary_table_median"+title_tag, "w") as f:
            make_table(f, all_data,
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
    significance_data(fitfuns, fitfunlabels, bd_type+["baseline"], runs, faults, generation, by_fitfun=True, load_existing=True,
                     title_tag="")

    #significance_data(fitfuns, fitfunlabels, bd_type+["baseline"], runs, faults, generation, by_fitfun=False, load_existing=True,
    #                 title_tag="")
    #make_significance_table(fitfunlabels, legend_labels, qed_index=-2,table_type="resilience")

    #gather_perturbation_results(datadir,generation,bd_type,fitfuns,faults,runs,history_type,perturbed=True)

    #gather_perturbation_results(datadir, generation, bd_type, fitfuns, faults, runs, history_type, perturbed=False)