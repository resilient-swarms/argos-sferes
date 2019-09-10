from sklearn.neighbors import KernelDensity

from dimensionality_plot import *
from perturbance_metrics import *
from NCD import *
from reduce_translated_archive import *
HOME_DIR = os.environ["HOME"]
RESULTSFOLDER="results"
from plots import *


import pickle


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
                get_help_data(BD_DIRECTORY+"/run"+str(run)+"_p"+str(fault)+"/results"+str(run),generation,history_type,translation_type)
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

def density_estimation(stats,x,xlabel, ylabel,
                   xlim, ylim,ax,title):
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
    #     Default is 'gaussian'.
    data = np.array([[xx, ss] for ss in stats for xx in x ])
    X=np.linspace(xlim[0],xlim[1],20)
    Y = np.linspace(ylim[0], ylim[1], 20)
    xy=np.array([[x,y] for y in Y for x in X] )
    s_x = xlim[1] - xlim[0]
    s_y = ylim[1] - ylim[0]
    bandwidth=min(s_x,s_y)/20.0
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
    #print("Z="+str(Z))
    # plot contours of the density
    levels = np.linspace(0, Z.max(), 25)
    print("plotting contour")
    ax.contourf(X, Y, Z, levels=levels, cmap=plt.cm.Reds)

    if title is not None:
        ax.set_title(title, fontsize=46)
    if ylim is not None:
        ax.set_ylim(ylim)
    if xlim is not None:
        ax.set_xlim(xlim)
    if xlabel is not None:
        ax.set_xlabel(xlabel, fontsize=36)
    if ylabel is not None:
        ax.set_ylabel(ylabel, fontsize=36)



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

def get_delta_P(non_perturbed_path,perturbed_path):
    _index, np_performance = get_best_individual(non_perturbed_path,add_performance=True)
    _index, p_performance = get_best_individual(perturbed_path,add_performance=True)
    return p_performance - np_performance


def gather_perturbation_results(datadir,generation,bd_type,fitfuns,faults,runs,history_type):
    centroids_sdbc=load_centroids("centroids/centroids_10_10.dat")
    centroids=load_centroids("centroids/centroids_10_3.dat")
    for bd in bd_type:
        for fitfun in fitfuns:
            title = fitfun + "range0.11"
            prefix = datadir + "/" + title + "/" + bd
            ncds, performances, nofaultperfs, euclids, categories= gather_perturbation_data(prefix, generation, faults,
                                                                                runs=runs, history_type=history_type,
                                                                                translation_type="sdbc",centroids=centroids_sdbc,get_NCD=False)
            _, _, _, relative_ents,_= gather_perturbation_data(prefix, generation, faults,
                                                               runs=runs, history_type=history_type,
                                                               translation_type="spirit",get_NCD=False)

            _, _, _ , _, categories_handcrafted = gather_perturbation_data(prefix, generation, faults,
                                                                                runs=runs, history_type=history_type,
                                                                                translation_type="handcrafted",centroids=centroids,get_NCD=False)
            dp_file,ncd_file, euclid_file, ent_file, category_file, category_h_file = filenames(fitfun,bd,history_type)
            pickle.dump((performances,nofaultperfs), open(dp_file, "wb"))
            pickle.dump(ncds, open(ncd_file, "wb"))
            pickle.dump(euclids, open(euclid_file, "wb"))
            pickle.dump(relative_ents, open(ent_file, "wb"))
            pickle.dump(categories, open(category_file, "wb"))
            pickle.dump(categories_handcrafted, open(category_h_file, "wb"))

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
    prefix= fitfun + bd + history_type
    return prefix + "_DeltaPs.pkl",prefix +"_ncds.pkl",\
           prefix +"_euclids.pkl",prefix +"_relativeEnts.pkl", \
          prefix +  "_categories.pkl", prefix +  "handcrafted_categories.pkl"

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
                       xlim=xlim, ylim=[0, 1], save_filename="results/FinalBDComp/NCD_DELTAP.pdf",
                       legend_labels=leg_labels, scatter=True, force=True,
                       ax=axs1[i],title=titles[i])

            createPlot(stats, x2, colors, markers, xlabel="performance", ylabel="$NCD$",
                       xlim=xlim2, ylim=[0, 1], save_filename="results/FinalBDComp/NCD_P.pdf",
                       legend_labels=leg_labels, scatter=True, force=True,
                       ax=axs2[i],title=titles[i])


        createPlot(stats2, x, colors, markers, xlabel="performance change", ylabel="Euclidian distance",
                   xlim=xlim, ylim=[0, 1], save_filename="results/FinalBDComp/Euclid_DELTAP.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs3[i],title=titles[i])

        createPlot(stats2, x2, colors, markers, xlabel="performance", ylabel="Euclidian distance",
                   xlim=xlim2, ylim=[0, 1], save_filename="results/FinalBDComp/Euclid_P.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs4[i],title=titles[i])

        createPlot(stats3, x, colors, markers, xlabel="performance change", ylabel="maximum variation distance",
                   xlim=xlim,ylim=[0,1.0], save_filename="results/FinalBDComp/MAXVAR_DELTAP.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs5[i],title=titles[i])

        createPlot(stats3, x2, colors, markers, xlabel="performance", ylabel="maximum variation distance",
                   xlim=xlim2, ylim=[0, 1.0], save_filename="results/FinalBDComp/MAXVAR_P.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs6[i],title=titles[i])

        createPlot(stats4, x, colors, markers, xlabel="performance change", ylabel="category",
                   xlim=xlim, ylim=[0, 10], save_filename="results/FinalBDComp/category_DELTAP.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs7[i],title=titles[i])

        createPlot(stats4, x2, colors, markers, xlabel="performance", ylabel="category",
                   xlim=xlim2, ylim=[0, 10], save_filename="results/FinalBDComp/category_P_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs8[i],title=titles[i])


        createPlot(stats5, x, colors, markers, xlabel="performance change", ylabel="category",
                   xlim=xlim, ylim=[0, 10], save_filename="results/FinalBDComp/category_DELTAP.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs9[i],title=titles[i])

        createPlot(stats5, x2, colors, markers, xlabel="performance", ylabel="category",
                   xlim=xlim2, ylim=[0, 10], save_filename="results/FinalBDComp/category_P_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs10[i],title=titles[i])
    finish_fig(fig1, "results/FinalBDComp/NCD_DELTAP.pdf")
    finish_fig(fig2, "results/FinalBDComp/NCD_P.pdf")
    finish_fig(fig3, "results/FinalBDComp/Euclid_DELTAP.pdf")
    finish_fig(fig4, "results/FinalBDComp/Euclid_P.pdf")
    finish_fig(fig5, "results/FinalBDComp/MAXVAR_DELTAP.pdf")
    finish_fig(fig6, "results/FinalBDComp/MAXVAR_P.pdf")
    finish_fig(fig7, "results/FinalBDComp/Category_DELTAP.pdf")
    finish_fig(fig8, "results/FinalBDComp/Category_P.pdf")
    finish_fig(fig9, "results/FinalBDComp/CategoryH_DELTAP.pdf")
    finish_fig(fig10, "results/FinalBDComp/CategoryH_P.pdf")

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
                   xlim=xlim, ylim=[0, 1], save_filename="results/FinalBDComp/NCD_DELTAP_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs1[i],title=titles[i])

        createPlot(stats, x2, colors, markers, xlabel="$P$", ylabel="$NCD$",
                   xlim=[0,1], ylim=[0, 1], save_filename="results/FinalBDComp/NCD_P_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs2[i],title=titles[i])


        createPlot(stats2, x, colors, markers, xlabel="$\Delta P$", ylabel="Euclidian distance",
                   xlim=xlim, ylim=[0, 1], save_filename="results/FinalBDComp/Euclid_DELTAP_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs3[i],title=titles[i])

        createPlot(stats2, x2, colors, markers, xlabel="$P$", ylabel="Euclidian distance",
                   xlim=[0,1], ylim=[0, 1], save_filename="results/FinalBDComp/Euclid_P_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs4[i],title=titles[i])

        createPlot(stats3, x, colors, markers, xlabel="$\Delta P$", ylabel="maximum variation distance",
                   xlim=xlim, ylim=[0, 1.0], save_filename="results/FinalBDComp/MAXVAR_DELTAP_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs5[i],title=titles[i])

        createPlot(stats3, x2, colors, markers, xlabel="$P$", ylabel="maximum variation distance",
                   xlim=[0,1], ylim=[0, 1.0], save_filename="results/FinalBDComp/MAXVAR_P_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs6[i],title=titles[i])

        createPlot(stats4, x, colors, markers, xlabel="$\Delta P$", ylabel="category",
                   xlim=xlim, ylim=[0, 10], save_filename="results/FinalBDComp/category_DELTAP_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs7[i],title=titles[i])

        createPlot(stats4, x2, colors, markers, xlabel="$P$", ylabel="category",
                   xlim=[0,1], ylim=[0, 10], save_filename="results/FinalBDComp/category_P_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs8[i],title=titles[i])

        createPlot(stats5, x, colors, markers, xlabel="performance change", ylabel="category",
                   xlim=xlim, ylim=[0, 10], save_filename="results/FinalBDComp/category_DELTAP.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs9[i],title=titles[i])

        createPlot(stats5, x2, colors, markers, xlabel="performance", ylabel="category",
                   xlim=[0,1], ylim=[0, 10], save_filename="results/FinalBDComp/category_P_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs10[i],title=titles[i])
        i += 1


    finish_fig(fig1, "results/FinalBDComp/NCD_DELTAP_desc.pdf")
    finish_fig(fig2, "results/FinalBDComp/NCD_P_desc.pdf")
    finish_fig(fig3, "results/FinalBDComp/Euclid_DELTAP_desc.pdf")
    finish_fig(fig4, "results/FinalBDComp/Euclid_P_desc.pdf")
    finish_fig(fig5, "results/FinalBDComp/MAXVAR_DELTAP_desc.pdf")
    finish_fig(fig6, "results/FinalBDComp/MAXVAR_P_desc.pdf")
    finish_fig(fig7, "results/FinalBDComp/Category_DELTAP_desc.pdf")
    finish_fig(fig8, "results/FinalBDComp/Category_P_desc.pdf")
    finish_fig(fig9, "results/FinalBDComp/CategoryH_DELTAP_desc.pdf")
    finish_fig(fig10, "results/FinalBDComp/CategoryH_P_desc.pdf")


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
            stats5.append(categories_h)
            x2.append(np.array(performances))
            x.append(np.array(dps)/np.array(nofaultperfs))
        #print(stats)
        #print(x)
        xlim2=xlim2_dict[fitfun]
        xlim=[-.25,.05]
        if plot_NCD:
            createPlot(stats, x, colors, markers, xlabel="proportional performance change", ylabel="$NCD$",
                       xlim=xlim, ylim=[0, 1], save_filename="results/FinalBDComp/NCD_DELTAP.pdf",
                       legend_labels=leg_labels, scatter=True, force=True,
                       ax=axs1[i],title=titles[i])

            createPlot(stats, x2, colors, markers, xlabel="performance", ylabel="$NCD$",
                       xlim=xlim2, ylim=[0, 1], save_filename="results/FinalBDComp/NCD_P.pdf",
                       legend_labels=leg_labels, scatter=True, force=True,
                       ax=axs2[i],title=titles[i])


        createPlot(stats2, x, colors, markers, xlabel="proportional performance change", ylabel="Euclidian distance",
                   xlim=xlim, ylim=[0, 1], save_filename="results/FinalBDComp/Euclid_DELTAP.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs3[i],title=titles[i])

        createPlot(stats2, x2, colors, markers, xlabel="performance", ylabel="Euclidian distance",
                   xlim=xlim2, ylim=[0, 1], save_filename="results/FinalBDComp/Euclid_P.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs4[i],title=titles[i])

        createPlot(stats3, x, colors, markers, xlabel="proportional performance change", ylabel="maximum variation distance",
                   xlim=xlim,ylim=[0,1.0], save_filename="results/FinalBDComp/MAXVAR_DELTAP.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs5[i],title=titles[i])

        createPlot(stats3, x2, colors, markers, xlabel="performance", ylabel="maximum variation distance",
                   xlim=xlim2, ylim=[0, 1.0], save_filename="results/FinalBDComp/MAXVAR_P.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs6[i],title=titles[i])

        createPlot(stats4, x, colors, markers, xlabel="proportional performance change", ylabel="category",
                   xlim=xlim, ylim=[0, 10], save_filename="results/FinalBDComp/category_DELTAP.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs7[i],title=titles[i])

        createPlot(stats4, x2, colors, markers, xlabel="performance", ylabel="category",
                   xlim=xlim2, ylim=[0, 10], save_filename="results/FinalBDComp/category_P_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs8[i],title=titles[i])
        createPlot(stats5, x, colors, markers, xlabel="performance change", ylabel="category",
                   xlim=xlim, ylim=[0, 10], save_filename="results/FinalBDComp/category_DELTAP.pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs9[i],title=titles[i])

        createPlot(stats5, x2, colors, markers, xlabel="performance", ylabel="category",
                   xlim=[0,1], ylim=[0, 10], save_filename="results/FinalBDComp/category_P_" + bd + ".pdf",
                   legend_labels=leg_labels, scatter=True, force=True,
                   ax=axs10[i],title=titles[i])
    finish_fig(fig1, "results/FinalBDComp/proportional_NCD_DELTAP.pdf")
    finish_fig(fig2, "results/FinalBDComp/proportional_NCD_P.pdf")
    finish_fig(fig3, "results/FinalBDComp/proportional_Euclid_DELTAP.pdf")
    finish_fig(fig4, "results/FinalBDComp/proportional_Euclid_P.pdf")
    finish_fig(fig5, "results/FinalBDComp/proportional_MAXVAR_DELTAP.pdf")
    finish_fig(fig6, "results/FinalBDComp/proportional_MAXVAR_P.pdf")
    finish_fig(fig7, "results/FinalBDComp/proportional_Category_DELTAP.pdf")
    finish_fig(fig8, "results/FinalBDComp/proportional_Category_P.pdf")
    finish_fig(fig9, "results/FinalBDComp/proportional_CategoryH_DELTAP.pdf")
    finish_fig(fig10, "results/FinalBDComp/proportional_CategoryH_P.pdf")


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
        #print(stats)
        #print(x)
        xlim = [-.25, .05]
        xlim2 = [0,1]
        ylim=[0,1]
        if plot_NCD:
            density_estimation(stats,x,xlabel="proportional performance change",
                           ylabel="$NCD$", xlim=xlim, ylim=ylim,ax=axs1[i],title=titles[i])
            density_estimation(stats,x2,xlabel="performance",
                           ylabel="$NCD$", xlim=xlim2, ylim=ylim,ax=axs2[i],title=titles[i])
        density_estimation(stats2, x,  xlabel="proportional performance change",
                           ylabel="Euclidian distance", xlim=xlim, ylim=ylim, ax=axs3[i], title=titles[i])
        density_estimation(stats2, x2,  xlabel="performance",
                           ylabel="Euclidian distance", xlim=xlim2, ylim=ylim, ax=axs4[i], title=titles[i])

        density_estimation(stats3, x,  xlabel="proportional performance change",
                           ylabel="maximum variation distance", xlim=xlim, ylim=ylim, ax=axs5[i], title=titles[i])
        density_estimation(stats3, x2,  xlabel="performance",
                           ylabel="maximum variation distance", xlim=xlim2, ylim=ylim, ax=axs6[i], title=titles[i])


        density_estimation(stats4, x,  xlabel="proportional performance change",
                           ylabel="category", xlim=xlim, ylim=[0,9.5], ax=axs7[i], title=titles[i])
        density_estimation(stats4, x2,  xlabel="performance",
                           ylabel="category", xlim=xlim2, ylim=[0,9.5], ax=axs8[i], title=titles[i])

        density_estimation(stats5, x,  xlabel="proportional performance change",
                           ylabel="category", xlim=xlim, ylim=[0,9.5], ax=axs9[i], title=titles[i])
        density_estimation(stats5, x2,  xlabel="performance",
                           ylabel="category", xlim=xlim2, ylim=[0,9.5], ax=axs10[i], title=titles[i])
        i += 1
    if plot_NCD:
        finish_fig(fig1, "results/FinalBDComp/Density_NCD_DELTAP_desc.pdf")
        finish_fig(fig2, "results/FinalBDComp/Density_NCD_P_desc.pdf")
    finish_fig(fig3, "results/FinalBDComp/Density_Euclid_DELTAP_desc.pdf")
    finish_fig(fig4, "results/FinalBDComp/Density_Euclid_P_desc.pdf")
    finish_fig(fig5, "results/FinalBDComp/Density_MAXVAR_DELTAP_desc.pdf")
    finish_fig(fig6, "results/FinalBDComp/Density_MAXVAR_P_desc.pdf")
    finish_fig(fig7, "results/FinalBDComp/Density_Category_DELTAP_desc.pdf")
    finish_fig(fig8, "results/FinalBDComp/Density_CategoryH_P_desc.pdf")
    finish_fig(fig9, "results/FinalBDComp/Density_CategoryH_DELTAP_desc.pdf")
    finish_fig(fig10, "results/FinalBDComp/Density_CategoryH_P_desc.pdf")



def perturbed_vs_unperturbed_best(fitfuns,bd_type,bd_labels,save_file,ylim):

    data=[]
    for i in range(len(fitfuns)):
        data.append([])
        for j in range(len(bd_type)):
            # get the best performances, fault vs no_fault
            dp_file, ncd_file = filenames(fitfuns[i], bd_type[j], history_type)
            performances, nofaultperfs = pickle.load(open(dp_file, "rb"))[i]
            data[i].append([nofaultperfs,performances])

    #make_boxplot_matrix(data, fitfuns, bd_labels, save_file, xlabs=["no perturb.","perturb."], ylab="performance",ylim=ylim)
    make_boxplot_pairswithin(data, fitfuns, bd_labels, save_file, xlabs=bd_labels, ylab="performance",
                        ylim=ylim)
def perturbed_vs_unperturbed_archive(fitfuns,bd_type,runs,faults,time,bd_labels,save_file,ylim):

    data=[]
    for i in range(len(fitfuns)):
        BD_dir = get_bd_dir(fitfuns[i])
        data.append([])
        for j in range(len(bd_type)):
            # get all the data from the archive: no fault
            nofaultperfs=np.array(list(get_combined_archive(BD_dir+"/"+bd_type[j]+"/FAULT_NONE",runs,"analysis"+str(time)+"_handcrafted.dat").values())).flatten()
            # join all the data from all the fault archives:
            performances=[]
            for fault in range(len(faults)):
                for run in runs:
                    temp=np.array(list(get_bin_performances_uniquearchive(BD_dir+"/"+bd_type[j]+"/run"+str(run)+"_p"+str(fault)+"/results"+str(run)+"/analysis"+str(time)+"_handcrafted.dat").values())).flatten()
                    performances=np.append(performances,temp)

            data[i].append([nofaultperfs,performances])

    #make_boxplot_matrix(data, fitfuns, bd_labels, save_file, xlabs=["no perturb.","perturb."], ylab="performance",ylim=ylim)
    make_boxplot_pairswithin(data, fitfuns, bd_labels, save_file, xlabs=bd_labels, ylab="performance",
                        ylim=ylim)





def significance_data(fitfuns,bd_type,runs,faults,time, by_fitfun=True):
    """

    performance: defined as the performance on all the perturbed environments
    transfer: defined as each individuals' drop in performance
    resilience: the best performance's drop in performance



    :param fitfuns:
    :param bd_type:
    :param runs:
    :param faults:
    :param time:
    :return:
    """
    performance_data = []
    transfer_data = []
    resilience_data = []

    for i in range(len(bd_type)):
        print(bd_type[i])
        performance_data.append([])
        transfer_data.append([])
        resilience_data.append([])
        for j in range(len(fitfuns)):
            print(fitfuns[j])
            BD_dir = get_bd_dir(fitfuns[j])
            # get all the data from the archive: no fault

            nofaultpath=BD_dir + "/" + bd_type[i] + "/FAULT_NONE/results"
            nofaultperfs = [np.array(list(get_ind_performances_uniquearchive(nofaultpath+str(run)+"/analysis" + str(time) + "_handcrafted.dat").values())).flatten() for run in runs]

            best_nofaultperfs = np.array([get_performance_data(nofaultpath+str(run), generation) for run in
                                          runs])
            # join all the data from all the fault archives:
            performances = []
            best_performances = []
            resilience = []
            transfer = []
            for fault in range(len(faults)):
                for r, run in enumerate(runs):
                    print(run)
                    path = BD_dir + "/" + bd_type[i] + "/run" + str(run) + "_p" + str(fault) + "/results" + str(
                        run) + "/analysis" + str(time) + "_handcrafted.dat"
                    temp = np.array(list(get_ind_performances_uniquearchive(path).values())).flatten()
                    performances = np.append(performances, temp)

                    maxind, best_performance = get_best_individual(path, add_performance=True)
                    best_performances = np.append(best_performances, best_performance)
                    # best performance vs best nofaultperf
                    resilience = np.append(resilience, (best_performance - best_nofaultperfs[r]) / nofaultperfs[r])
                    # all performances vs all nofaultperformances
                    transfer = np.append(transfer, (temp - nofaultperfs[r]) / nofaultperfs[r])

            if by_fitfun:
                performance_data[i].append((np.mean(performances),np.std(performances)))
                transfer_data[i].append((np.mean(transfer),np.std(transfer)))
                resilience_data[i].append((np.mean(resilience),np.std(resilience)))
            else:
                performance_data[i] = np.append(performance_data[i],performances)
                transfer_data[i] = np.append(transfer_data[i], transfer)
                resilience_data[i] = np.append(resilience_data[i],resilience)
        if not by_fitfun:
            performance_data[i] = (np.mean(performance_data[i]),np.std(performance_data[i]))
            transfer_data[i] = (np.mean(transfer_data[i]), np.std(transfer_data[i]))
            resilience_data[i] = (np.mean(resilience_data[i]), np.std(resilience_data[i]))
    return performance_data, transfer_data, resilience_data

if __name__ == "__main__":
    #test_NCD(num_agents=10, num_trials=10, num_ticks=100, num_features=8)

    faults=range(40)
    F=len(faults)
    runs=range(1,6)
    bd_type = ["history","Gomes_sdbc_walls_and_robots_std","cvt_rab_spirit","environment_diversity"]  # legend label
    legend_labels = ["handcrafted","SDBC","SPIRIT","QED"]  # labels for the legend
    fitfuns = ["Aggregation","Dispersion","DecayCoverage"]
    #baseline_performances = {"Aggregation":0.9,"Dispersion":0.2, "Flocking":0.2}
    colors = ["C" + str(i) for i in range(len(bd_type))]
    markers = [(2, 1, 0), (3, 1, 0),(2, 1, 1), (3, 1, 1)]

    datadir= HOME_DIR + "/Data/ExperimentData"
    generation="10000"
    history_type="xy"
    #datadir, generation, bd_type, fitfuns, faults, runs, history_type)
    #gather_perturbation_results(datadir, generation, bd_type, fitfuns, faults,runs=range(1,6),history_type=history_type)
    #gather_category_results(bd_type, fitfuns, faults, runs=[1])

    #plot_by_fitfun(legend_labels,titles=fitfuns,plot_NCD=False)
    #plot_by_descriptor(fitfuns,titles=legend_labels,xlim=[-0.15,0.01])
    #plot_proportional_byfitfun(legend_labels,titles=fitfuns,plot_NCD=False)

    #plot_density_bydescriptor(titles=legend_labels,plot_NCD=False)
    time=10000
    #perturbed_vs_unperturbed_archive(fitfuns, bd_type, runs,faults,time,legend_labels,"boxplots_all.pdf",ylim=[0,1])
    #significance_data(fitfuns, bd_type, runs, faults, time, by_fitfun=True)
    significance_data(fitfuns, bd_type, runs, faults, time, by_fitfun=False)