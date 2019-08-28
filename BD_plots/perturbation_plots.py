
from dimensionality_plot import *
from NCD import *
HOME_DIR = os.environ["HOME"]
RESULTSFOLDER="results"
from plots import *

import pickle

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
    bd = np.array(get_combined_archive(data_path, runs, archive_file_path,by_bin=False,include_val=False))
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

def get_delta_P(non_perturbed_path,perturbed_path):
    _index, np_performance = get_best_individual(non_perturbed_path,add_performance=True)
    _index, p_performance = get_best_individual(perturbed_path,add_performance=True)
    return p_performance - np_performance


def gather_perturbation_results(datadir,generation,bd_type,fitfuns,faults,runs,history_type):
    for bd in bd_type:
        ncds_list = []
        delta_p_list = []

        for fitfun in fitfuns:
            title = fitfun + "range0.11"
            prefix = datadir + "/" + title + "/" + bd
            ncds, performances, nofaultperfs= gather_perturbation_data(prefix, generation, faults, runs=runs, history_type=history_type)

            ncds_list.append(ncds)
            delta_p_list.append((performances,nofaultperfs))
            dp_file,ncd_file = filenames(fitfun,bd,history_type)
            pickle.dump(delta_p_list, open(dp_file, "wb"))
            pickle.dump(ncds_list, open(ncd_file, "wb"))


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
    return fitfun + bd + history_type + "_DeltaPs.pkl",fitfun+ bd + history_type +"_ncds.pkl"

def plot_by_fitfun():
    i = 0
    for fitfun in fitfuns:
        stats = []
        x = []
        for bd in bd_type:
            dp_file, ncd_file = filenames(fitfun, bd, history_type)
            performances , nofaultperfs = pickle.load(open(dp_file, "rb"))[i]
            dps = np.array(performances) - np.array(nofaultperfs)
            ncds = pickle.load(open(ncd_file, "rb"))[i]
            stats.append(ncds)
            x.append(np.array(dps))
        print(stats)
        print(x)

        createPlot(stats, x, colors, markers, xlabel="$\Delta P$", ylabel="$NCD$",
                   xlim=[-0.45, 0.05], ylim=[0, 2.0], save_filename="results/FinalBDComp/NCD" + fitfun + ".pdf",
                   legend_labels=plot_titles, scatter=True, force=True)
        i += 1


def perturbed_vs_unperturbed_archive():
    i = 0
    for fitfun in fitfuns:
        stats = []
        x = []
        for bd in bd_type:
            dp_file, ncd_file = filenames(fitfun, bd, history_type)
            performances, nofaultperfs = pickle.load(open(dp_file, "rb"))[i]
            dps = np.array(performances) - np.array(nofaultperfs)
            ncds = pickle.load(open(ncd_file, "rb"))[i]
            stats.append(ncds)
            x.append(np.array(dps))
        print(stats)
        print(x)

        createPlot(stats, x, colors, markers, xlabel="$\Delta P$", ylabel="$NCD$",
                   xlim=[-0.45, 0.05], ylim=[0, 2.0], save_filename="results/FinalBDComp/NCD" + fitfun + ".pdf",
                   legend_labels=plot_titles, scatter=True, force=True)
        i += 1

if __name__ == "__main__":
    #test_NCD(num_agents=10, num_trials=10, num_ticks=100, num_features=8)

    faults=range(5)
    F=len(faults)
    bd_type = ["history","Gomes_sdbc_walls_and_robots_std","cvt_rab_spirit","environment_diversity"]  # legend label
    plot_titles = ["handcrafted","SDBC","SPIRIT","QED"]  # labels for the legend
    fitfuns = ["Aggregation","Dispersion", "Flocking"]
    #baseline_performances = {"Aggregation":0.9,"Dispersion":0.2, "Flocking":0.2}
    colors = ["C" + str(i) for i in range(len(bd_type))]
    markers = [(2, 1, 0), (3, 1, 0),(2, 1, 1), (3, 1, 1)]

    datadir= HOME_DIR + "/Data/ExperimentData"
    generation="5000"
    history_type="sa"
    gather_perturbation_results(datadir, generation, bd_type, fitfuns, faults,runs=[1,2],history_type=history_type)
    #gather_category_results(bd_type, fitfuns, faults, runs=[1])

    plot_by_fitfun()