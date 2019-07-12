

import numpy as np
from process_archive_data import *
from Utils.Plots.plots import createPlot
# behavioural metrics defined in Mouret, J., & Clune, J. (2015). Illuminating search spaces by mapping elites. 1–15.
import sys,os
HOME_DIR = os.environ["HOME"]
RESULTSFOLDER="results"
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



def development_plots(runs,times,BD_directory,title_tag):

    # bd_type = ["history","cvt_mutualinfo","cvt_mutualinfoact","cvt_spirit"]  #legend label
    #
    # legend_labels=["handcrafted","mutualinfo","mutualinfoact","spirit"]  # labels for the legend
    # colors=["C"+str(i) for i in range(len(bd_type))]  # colors for the lines
    # # (numsides, style, angle)
    # markers=[(3,1,0), (3,2,0),(4,1,0),(4,2,0)] # markers for the lines
    # bd_shapes = [32**2, 1000,1000,1000]  # shape of the characterisation
    # y_labels=["global_performance","global_reliability","precision","coverage"]

    bd_type = ["Gomes_sdbc_walls_and_robots_std","multiagent_spirit"]  #legend label

    legend_labels=["SDBC","multiagent_spirit"]  # labels for the legend
    colors=["C"+str(i) for i in range(len(bd_type))]  # colors for the lines
    # (numsides, style, angle)
    markers=[(3,1,0),(3,2,0)] # markers for the lines
    bd_shapes = [5000,5000]  # shape of the characterisation
    y_labels=["global_performance","global_reliability","precision","coverage"]


    boxes=[(.20,.20),(.20,.20),(0.20,0.20),(0.20,0.20)] # where to place the legend box
    y_bottom={ylabel:[[] for i in bd_type] for ylabel in y_labels}
    y_mid = {ylabel: [[] for i in bd_type]  for ylabel in y_labels}
    y_top = {ylabel: [[] for i in bd_type]  for ylabel in y_labels}
    for i in range(len(bd_type)):
        for time in times:
            archive_file="archive_" + str(time) + ".dat"

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
        createPlot(y_mid[label],x_values=np.array(times),colors=colors,markers=markers,xlabel="generations",ylabel=label.replace("_"," "),ylim=[0.0,0.1],
                   save_filename=RESULTSFOLDER+"/"+title_tag+label+".pdf",legend_labels=legend_labels,
                   xlim=None,xscale="linear",yscale="linear",
               legendbox=boxes[j],annotations=[],xticks=[],yticks=[],task_markers=[],scatter=False,
               legend_cols=1,legend_fontsize=26,legend_indexes=[],additional_lines=[],index_x=[],
               xaxis_style="plain",y_err=[],force=True,fill_between=(y_bottom[label],y_top[label]))
        j+=1


if __name__ == "__main__":
    
    
    runs=5

    #global_performances(HOME_DIR+"/Desktop/history_obstacles", runs, "archive_900.dat",1.0)
    #global_performances(HOME_DIR+"/argos-sferes/experiments/data/MeanSpeed/sdbc_walls_and_robots", runs, "archive_900.dat", 1.0)


    #global_reliabilities(HOME_DIR+"/Desktop/history_obstacles", runs, "archive_900.dat")
    #global_reliabilities(HOME_DIR+"/argos-sferes/experiments/data/MeanSpeed/sdbc_walls_and_robots", runs, "archive_900.dat")
    
    #precisions(HOME_DIR+"/Desktop/history_obstacles", runs, "archive_900.dat")
   # precisions(HOME_DIR+"/argos-sferes/experiments/data/MeanSpeed/sdbc_walls_and_robots", runs, "archive_900.dat")

    #coverages((3,10),HOME_DIR+"/Desktop/history_obstacles", runs, "archive_900.dat")
   # coverages((6, 10),HOME_DIR+"/argos-sferes/experiments/data/MeanSpeed/sdbc_walls_and_robots", runs, "archive_900.dat")


    data_dir=HOME_DIR+"/Data/dataNEW/datanew"

    #development_plots(runs=5,times=range(0,1100,100),BD_directory=data_dir+"/Coveragerange11",title_tag="range11")

    fitfuns= ["DecayCoverage","DecayBorderCoverage","Aggregation","Dispersion","Flocking"]

    for fitfun in fitfuns:
        title=fitfun+"range11"
        development_plots(runs=range(1,6), times=range(0,150, 10), BD_directory=data_dir + "/"+title,title_tag=fitfun)