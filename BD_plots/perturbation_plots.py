import zlib
import tarfile
from dimensionality_plot import *
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


def NCD_perturbation_plot(archive_path):
    pass

def get_help_data(directory,history_type,runs):
    max_performance = -float("inf")
    max_run = None
    max_indiv = None
    for run in runs:
        file = directory+str(run)+"/analysis_sdbc.dat"
        best_indiv, performance = get_best_individual(file,add_performance=True)
        if performance > max_performance:
            max_performance = performance
            max_run = run
            max_indiv = best_indiv
    history_file = directory + str(max_run) + "/" + history_type + "_history" + str(max_indiv)
    return history_file, max_performance


def gather_NCDs(BD_DIRECTORY,faults, runs, history_type="sa"):
    """
    use existing state/observation-action trajectory files and calculate pair-wise NCDs for all individuals within the sa;e run
    the average is returned
    :return:
    """
    assert faults[0]=="FAULT_NONE"

    directories=[ BD_DIRECTORY+"/"+faults[f] + "/results" for f in range(len(faults)) ]
    ncds = []
    delta_ps=[]
    history_comp, performance_comp = get_help_data(directories[0],history_type,runs)

    for i in range(1,len(directories)):
        history_file, performance = get_help_data(directories[i], history_type, runs)
        # get delta_p
        delta_p = performance - performance_comp
        print("performance difference: " +str(delta_p))
        delta_ps.append(delta_p)
        # get ncd
        ncd = NCD(history_comp,history_file)
        ncds.append(ncd)



    return ncds, delta_ps

def read_history_file(filename,from_gz=True):
    if from_gz:
        print("opening "+str(filename)+".tar.gz")
        tar = tarfile.open(filename+".tar.gz")
        member = tar.getmembers()[0]  # assumes only a single file in the archive
        f = tar.extractfile(member)
        content=f.read()
        tar.close()
        return content
    else:
        x = open(filename, 'rb').read()  # file 1
    return x

def NCD(file1,file2, from_gz=True):
    print("getting NCDs:")
    print(file1)
    x=read_history_file(file1,from_gz)
    print(file2)
    y=read_history_file(file2,from_gz)
    x_y = x + y  # concatenation

    x_c = zlib.compress(x)
    y_c = zlib.compress(y)
    x_y_c = zlib.compress(x_y)

    enum = len(x_y_c) - min(len(x_c), len(y_c))
    denom = max(len(x_c), len(y_c))
    NCD =  enum / denom
    print("NCD =%.3f"%(NCD))

def gather_perturbation_results(bd_type,fitfuns,faults,runs):
    for bd in bd_type:
        data_dir = HOME_DIR + "/DataFinal/datanew"

        ncds_list = []
        delta_p_list = []

        for fitfun in fitfuns:
            title = fitfun + "range11"
            prefix = data_dir + "/" + title + "/" + bd
            ncds, delta_p = gather_NCDs(prefix, faults, runs=runs, history_type="xy")

            ncds_list.append(ncds)
            delta_p_list.append(delta_p)
        dp_file,ncd_file = filenames(fitfun,bd)
        pickle.dump(delta_p_list, open(dp_file, "wb"))
        pickle.dump(ncds_list, open(ncd_file, "wb"))


def gather_category_results(bd_type, fitfuns, faults, runs):
    for bd in bd_type:
        data_dir = HOME_DIR + "/DataFinal/datanew"

        categories_list = []
        delta_p_list = []

        for fitfun in fitfuns:
            title = fitfun + "range11"
            prefix = data_dir + "/" + title + "/" + bd
            pca = get_pca(prefix,[1],"archive_1000.dat",["SDBC"+str(i) for i in range(10)])

            for run in runs:
                for fault in faults:
                    maxind, maxf, maxbd = get_best_individual(prefix + "/results" + str(run) + "/analysis_sdbc.dat")
                result = get_pca_result(pca,[maxbd],bins=3)

                pickle.dump(ncds_list, open(ncd_file, "wb"))
def filenames(fitfun,bd):
    return fitfun + bd + "_DeltaPs.pkl",fitfun+ bd + "_ncds.pkl"
if __name__ == "__main__":

    faults=["FAULT_NONE","FAULT_PROXIMITYSENSORS_SETMIN","FAULT_PROXIMITYSENSORS_SETMAX","FAULT_PROXIMITYSENSORS_SETRANDOM",
            "FAULT_ACTUATOR_LWHEEL_SETHALF", "FAULT_ACTUATOR_RWHEEL_SETHALF", "FAULT_ACTUATOR_BWHEELS_SETHALF"]
    F=len(faults)
    bd_type = ["environment_diversity"]  # legend label
    plot_titles = ["QED"]  # labels for the legend
    fitfuns = ["Aggregation","Dispersion", "DecayCoverage", "DecayBorderCoverage", "Flocking"]
    colors = ["C" + str(i) for i in range(len(bd_type))]
    markers = [(2, 1, 0), (3, 1, 0)]

    gather_perturbation_results(bd_type, fitfuns, faults,runs=[1])
    #gather_category_results(bd_type, fitfuns, faults, runs=[1])

    i=0
    for fitfun in fitfuns:
        stats=[]
        x=[]
        for bd in bd_type:
            dp_file, ncd_file = filenames("", bd)
            dps = pickle.load(open(dp_file,"rb"))[i]
            ncds= pickle.load(open(ncd_file,"rb"))[i]
            stats.append(ncds)
            x.append(np.array(dps))
        createPlot(stats,x,colors,markers,xlabel="$\Delta P$",ylabel="$NCD$",
                   xlim=[-0.05,0.05], ylim=[0,2.0],save_filename="perturbationresults/NCD"+fitfun+".png",legend_labels=plot_titles,scatter=True,force=True)
        i+=1