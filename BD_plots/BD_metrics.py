

import numpy as np
from process_archive_data import *
from Utils.Plots.plots import createPlot
from dimensionality_plot import *
# behavioural metrics defined in Mouret, J., & Clune, J. (2015). Illuminating search spaces by mapping elites. 1–15.
import sys,os
import lzma
HOME_DIR = os.environ["HOME"]
RESULTSFOLDER="results"


def scatter_plot(x,y,colors,area,filename):
    plt.scatter(x, y, s=area, c=colors, alpha=0.5)
    plt.savefig(filename)

def phenotype_perturbation_plot(data_path, runs, archive_file_path, bd_labels):
    """
    scatterplot of the phenotypes of solutions after perturb. as a function of the drop in fitness by the perturb.
    :return:
    """
    # get all datapoints' corresponding categories
    categories = phenotype_categories(data_path, runs, archive_file_path, bd_labels, components=2)

def get_delta_P(non_perturbed_path,perturbed_path, conversion_func):
    np_performances = np.array([conversion_func(fitness) for fitness in get_bin_performances(non_perturbed_path).values()])
    p_performances = np.array([conversion_func(fitness) for fitness in get_bin_performances(perturbed_path).values()])
    return p_performances - np_performances


def NCD_perturbation_plot(archive_path):
    pass

def gather_NCDs():
    """
    use existing state/observation-action trajectory files and calculate pair-wise NCDs for all individuals within the sa;e run
    the average is returned
    :return:
    """
    individuals = get_individuals(archive_path)
    result = []
    for p1 in range(len(individuals)):
        for p2 in range(p1 + 1, len(individuals)):
            result.append()

def NCD(file1,file2):
    x = open(file1, 'rb').read()  # file 1
    y = open(file2, 'rb').read()  # file 2
    x_y = x + y  # concatenation

    x_c = lzma.compress(x)
    y_c = lzma.compress(y)
    x_y_c = lzma.compress(x_y)

    enum = len(x_y_c) - min(len(x_c), len(y_c))
    denom = max(len(x_c), len(y_c))
    return  enum / denom


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
def phenotype_categories(data_path, runs, archive_file_path,bd_labels,components=2,bins=3):
    #bd = np.array(get_combined_archive(data_path, runs, archive_file_path,by_bin=False,include_val=False))
    raise Exception("need to impleement phenotype")
    phenotype = []
    df = pd.DataFrame(phenotype, columns=bd_labels)
    pca = PCA(n_components=components)
    pca_result = pca.fit_transform(df[bd_labels].values)
    minima = np.min(pca_result,axis=0) # get the minimum for each dimension
    maxima = np.max(pca_result,axis=0) # get the maximum for each dimension
    bin_sizes = (maxima - minima) / float(bins)

    pheno_cat = []
    for datapoint in pca_result:
        pheno_cat.append(bin_single_point(datapoint, minima, bins, bin_sizes))

    return pheno_cat


def global_performances(BD_directory, runs, archive_file_path, max_performance,conversion_func):
    stats = []
    for run in runs:
        stats.append(_global_performance(BD_directory, run, archive_file_path, max_performance,conversion_func))
    print("global performances: " + str(stats))
    return stats

def _global_performance(BD_directory, run,archive_file_path,max_performance,conversion_func=None):
    """
    For each run, the single highest- performing
    solution found by that algorithm anywhere in the search space
    divided by the highest performance possi- ble in that domain.
    If it is not known what the maximum theoretical performance is,
    as is the case for all of our do- mains,
    it can be estimated by dividing by the highest performance found by any algorithm in any run. ( BUT avoid estimation if you can)
    This measure is the traditional, most common way to evaluate optimization algorithms.
    This measure is the traditional, most common way to evaluate optimization algorithms.
    One can also measure whether any illumination algorithm also performs well on this measurement.
    Both the ideal optimization algorithm and the ideal illumination
    algorithm are expected to perform perfectly on this measure

    :param BD_directory: directory in which all the runs of a BD are located,
            e.g. ~/Desktop/history_obstacles
    :param experiment_file_path: relative path from the BD_directory to the archive file
    """
    path=get_archive_filepath(BD_directory, run, archive_file_path)
    if conversion_func is not None:
        all_performances = [conversion_func(fitness) for fitness in get_bin_performances(path).values()]
    else:
        all_performances = [fitness for fitness in get_bin_performances(path).values()]
    return max(all_performances)/max_performance


def avg_performances(BD_directory, runs, archive_file_path, max_performance,conversion_func):
    stats = []
    for run in runs:
        stats.append(_avg_performance(BD_directory, run, archive_file_path, max_performance,conversion_func))
    print("avg performances: " + str(stats))
    return stats

def _avg_performance(BD_directory, run,archive_file_path,max_performance,conversion_func=None):
    """
    For each run, the single highest- performing
    solution found by that algorithm anywhere in the search space
    divided by the highest performance possi- ble in that domain.
    If it is not known what the maximum theoretical performance is,
    as is the case for all of our do- mains,
    it can be estimated by dividing by the highest performance found by any algorithm in any run. ( BUT avoid estimation if you can)
    This measure is the traditional, most common way to evaluate optimization algorithms.
    This measure is the traditional, most common way to evaluate optimization algorithms.
    One can also measure whether any illumination algorithm also performs well on this measurement.
    Both the ideal optimization algorithm and the ideal illumination
    algorithm are expected to perform perfectly on this measure

    :param BD_directory: directory in which all the runs of a BD are located,
            e.g. ~/Desktop/history_obstacles
    :param experiment_file_path: relative path from the BD_directory to the archive file
    """
    path=get_archive_filepath(BD_directory, run, archive_file_path)
    if conversion_func is not None:
        mean_performances = [conversion_func(fitness) for fitness in get_bin_performances(path).values()]
    else:
        mean_performances = [fitness for fitness in get_bin_performances(path).values()]
    return np.mean(mean_performances)/max_performance

def global_reliabilities(BD_directory,runs,archive_file_path):
    #bins=get_bins(bd_shape)

    combined_archive=get_combined_archive(BD_directory, runs, archive_file_path)
    stats = []
    for run in runs:
        stats.append(_global_reliability(combined_archive,BD_directory, run, archive_file_path))
    print("global reliabilities: "+str(stats))
    return stats

def _global_reliability(combined_archive,BD_directory, run, archive_file_path):

    """
    For each run, the average across all cells of the highest-performing solution the algorithm found for each cell
    (0 if it did not produce a solution in that cell)
    divided by the best known performance for that cell as found by any run of any algorithm.
    Cells for which no solution was found by any run of any algorithm are not included in the calculation
    (to avoid dividing by zero, and because it may not be possible to fill such cells and algorithms
     thus should not be penalized for not doing so).

     Based on the formal equation:


    :param BD_directory: directory in which all the runs of a BD are located,
            e.g. ~/Desktop/history_obstacles
    :params runs, the number of runs
    :param archive_file_path, the relative path from the BD_directory to the archive file
    :return:
    """
    path = get_archive_filepath(BD_directory, run, archive_file_path)
    all_non_empty_performances = get_bin_performances(path)

    cell_performances = []

    for bin , archive_perfs in combined_archive.items():  # take the mean across the cells that are not empty across runs
        performance = all_non_empty_performances.get(bin, 0.0)
        max_performance = max(archive_perfs)
        assert performance <= max_performance
        cell_performances.append(performance/max_performance)
    mean = np.mean(cell_performances)
    return mean

def precisions(BD_directory,runs,archive_file_path):
    combined_archive=get_combined_archive(BD_directory, runs, archive_file_path)
    stats = []
    for run in runs:
        stats.append(_precision(combined_archive,BD_directory, run, archive_file_path))
    print("precisions"+str(stats))
    return stats
def _precision(combined_archive, BD_directory, run, archive_file_path):

    """
    Same as global reliability, but for each run, the normalized performance
    is averaged only for the cells that were filled by that algo- rithm in that run.


    :param BD_directory: directory in which all the runs of a BD are located,
            e.g. ~/Desktop/history_obstacles
    :params runs, the number of runs
    :param archive_file_path, the relative path from the BD_directory to the archive file
    :return:
    """
    path = get_archive_filepath(BD_directory, run, archive_file_path)
    all_non_empty_performances = get_bin_performances(path)

    cell_performances = []

    for bin , archive_perf in all_non_empty_performances.items():  # take the mean across the cells that are not empty *in this run*
        performance = archive_perf
        cp=combined_archive[bin]
        max_performance = max(cp)
        assert performance <= max_performance
        cell_performances.append(performance/max_performance)
    mean = np.mean(cell_performances)
    return mean
def coverages(bd_shape,BD_directory, runs, archive_file_path):
    stats = []
    for run in runs:
        stats.append(_coverage(bd_shape,BD_directory, run, archive_file_path))
    print("global coverages"+str(stats))
    return stats

def _coverage(bd_shape,BD_directory, run, archive_file_path):
    """
    Measures how many cells of the feature space a run of an algorithm 
    is able to fill of the total number that are possible to fill.
    :return: 
    """

    path = get_archive_filepath(BD_directory, run, archive_file_path)
    all_non_empty_performances = get_bin_performances(path)
    num_filled=len(all_non_empty_performances)
    max_num_filled=get_bins(bd_shape)
    return float(num_filled)/float(max_num_filled)

def absolutecoverages(bd_shape,BD_directory, runs, archive_file_path):
    stats = []
    for run in runs:
        stats.append(_absolutecoverage(bd_shape,BD_directory, run, archive_file_path))
    print("global coverages"+str(stats))
    return stats

def _absolutecoverage(bd_shape,BD_directory, run, archive_file_path):
    """
    Measures how many cells of the feature space a run of an algorithm
    is able to fill
    :return:
    """

    path = get_archive_filepath(BD_directory, run, archive_file_path)
    all_non_empty_performances = get_bin_performances(path)
    num_filled=len(all_non_empty_performances)
    return float(num_filled)




def add_boxplotlike_data(stats, y_bottom,y_mid,y_top, y_label,method_index):
    """
    return x.25,x.50 and x.75
    :return:
    """
    x1,x2,x3=tuple(np.quantile(stats, q=[.25, .50, .75]))
    y_bottom[y_label][method_index].append(x1)
    y_mid[y_label][method_index].append(x2)
    y_top[y_label][method_index].append(x3)

def convert_CoverageFitness(fitness,grid_size=0.1212,max_velocity=0.10, time_per_trial=120, total_cells=1090):
    """
    :param fitness: the coverage fitness e.g. 0.10 means 10% of all cells are visited
    :param grid_size: e.g. each cell 0.14 m
    :param max_velocity: e.g. maximal velocity of thymio is 0.10m/s
    :param time_per_trial: e.g. 120 seconds
    :param arena_size: e.g. (4,4) is a 4-by-4 arena
    :return:
    """
    max_cells_per_second = max_velocity/grid_size
    max_cells_per_trial = np.ceil(max_cells_per_second*time_per_trial)

    visited_cells = np.ceil(total_cells*fitness)  #e.g. 160 cells, then fitness=0.1  means 16 cells visited
    return visited_cells/max_cells_per_trial  # coverage now means visited cells compared to the maximum possible

def print_best_individuals(BD_dir,outfile, number,generation):
    solutions, indexes = get_best_individuals(BD_dir, range(1,6), "/archive_"+str(generation)+".dat",number,criterion="fitness")
    with open(outfile+"fitness.txt", 'w') as f:
        i=0
        for key,value in solutions.items():
            f.write("%s %s %.3f \n "%(indexes[i], str(key),value[0]))
            i+=1
        f.flush()



    solutions, indexes = get_best_individuals(BD_dir, range(1, 6), "/archive_"+str(generation)+".dat", number, criterion="diversity")
    with open(outfile + "diversity.txt", 'w') as f:
        i = 0
        for array in solutions:
            f.write("%s %s %.3f \n" % (indexes[i], array[0:-1], array[-1]))
            i += 1

def development_plots(runs,times,BD_directory,title_tag):

    # bd_type = ["history","cvt_mutualinfo","cvt_mutualinfoact","cvt_spirit"]  #legend label
    #
    # legend_labels=["handcrafted","mutualinfo","mutualinfoact","spirit"]  # labels for the legend
    # colors=["C"+str(i) for i in range(len(bd_type))]  # colors for the lines
    # # (numsides, style, angle)
    # markers=[(3,1,0), (3,2,0),(4,1,0),(4,2,0)] # markers for the lines
    # bd_shapes = [32**2, 1000,1000,1000]  # shape of the characterisation
    # y_labels=["global_performance","global_reliability","precision","coverage"]

    bd_type = ["Gomes_sdbc_walls_and_robots_std","environment_diversity"]  #legend label
    legend_labels=["SDBC","QED"]  # labels for the legend
    colors=["C"+str(i) for i in range(len(bd_type))]  # colors for the lines
    # (numsides, style, angle)
    markers=[(3,1,0),(3,2,0)] # markers for the lines
    bd_shapes = [5000,5000]  # shape of the characterisation
    y_labels=["absolute_coverage","average_performance","global_performance","global_reliability","precision","coverage"]


    boxes=[(.20,.20),(.20,.20),(.20,.20),(.20,.20),(0.20,0.20),(0.20,0.20)] # where to place the legend box
    y_bottom={ylabel:[[] for i in bd_type] for ylabel in y_labels}
    y_mid = {ylabel: [[] for i in bd_type]  for ylabel in y_labels}
    y_top = {ylabel: [[] for i in bd_type]  for ylabel in y_labels}


    for i in range(len(bd_type)):
        for time in times:
            archive_file="archive_" + str(time) + ".dat"


            abs_coverage=absolutecoverages(bd_shapes[i], BD_directory+"/"+bd_type[i], runs, archive_file)
            add_boxplotlike_data(abs_coverage, y_bottom, y_mid, y_top, y_label="absolute_coverage",method_index=i)


            avg_perform = avg_performances(BD_directory+"/"+bd_type[i], runs, archive_file , 1.0,
                                                 conversion_func=None)
            add_boxplotlike_data(avg_perform, y_bottom, y_mid, y_top, y_label="average_performance",method_index=i)


            global_perform = global_performances(BD_directory+"/"+bd_type[i], runs, archive_file , 1.0,
                                                 conversion_func=None)
            add_boxplotlike_data(global_perform, y_bottom, y_mid, y_top, y_label="global_performance",method_index=i)
            global_reliability = global_reliabilities(BD_directory+"/"+bd_type[i], runs, archive_file)
            add_boxplotlike_data(global_reliability, y_bottom, y_mid, y_top, y_label="global_reliability",method_index=i)

            precision=precisions(BD_directory+"/"+bd_type[i], runs, archive_file)
            add_boxplotlike_data(precision, y_bottom, y_mid, y_top, y_label="precision",method_index=i)

            coverage=coverages(bd_shapes[i], BD_directory+"/"+bd_type[i], runs, archive_file)
            add_boxplotlike_data(coverage, y_bottom, y_mid, y_top, y_label="coverage",method_index=i)
    j=0
    for label in y_labels:
        ylim=[0,5000] if label == "absolute_coverage"   else [0.0,1.0]
        createPlot(y_mid[label],x_values=np.array(times),colors=colors,markers=markers,xlabel="generations",ylabel=label.replace("_"," "),ylim=ylim,
                   save_filename=RESULTSFOLDER+"/"+title_tag+label+".pdf",legend_labels=legend_labels,
                   xlim=None,xscale="linear",yscale="linear",
               legendbox=boxes[j],annotations=[],xticks=[],yticks=[],task_markers=[],scatter=False,
               legend_cols=1,legend_fontsize=26,legend_indexes=[],additional_lines=[],index_x=[],
               xaxis_style="plain",y_err=[],force=True,fill_between=(y_bottom[label],y_top[label]))
        j+=1


if __name__ == "__main__":
    
    
    runs=5

    scatter_plot()







    fitfuns= ["Aggregation","Dispersion","DecayCoverage","DecayBorderCoverage","Flocking"]

    for fitfun in fitfuns:
        data_dir = HOME_DIR + "/DataFinal/datanew"
        title=fitfun+"range11"
        print_best_individuals(
            BD_dir="/home/david/DataFinal/datanew/"+fitfun+"range11/Gomes_sdbc_walls_and_robots_std",
            outfile="best_solutions_"+fitfun+"NOCORRECT", number=10, generation=1200)
        development_plots(runs=range(1,6), times=range(0,1100, 100), BD_directory=data_dir + "/"+title,title_tag=fitfun+"NOCORRECT")




    # for fitfun in fitfuns:
    #     data_dir = HOME_DIR + "/DataFinal/coll"
    #     title=fitfun+"range11"
    #     print_best_individuals(
    #         BD_dir="/home/david/DataFinal/coll/"+fitfun+"range11/Gomes_sdbc_walls_and_robots_std",
    #         outfile="best_solutions_"+fitfun+"COLLISIONSTOP", number=10, generation=1200)
    #     development_plots(runs=range(1,6), times=range(0,1250, 50), BD_directory=data_dir + "/"+title,title_tag=fitfun+"COLLISIONSTOP",)