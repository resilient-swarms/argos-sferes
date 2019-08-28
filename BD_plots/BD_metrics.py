from plots import *
from dimensionality_plot import *
# behavioural metrics defined in Mouret, J., & Clune, J. (2015). Illuminating search spaces by mapping elites. 1–15.
import os

HOME_DIR = os.environ["HOME"]
RESULTSFOLDER="results"

import copy



def get_all_performances(path,conversion_func=None,from_fitfile=False):
    if conversion_func is not None:
        all_performances = [conversion_func(fitness) for fitness in get_bin_performances_uniquearchive(path,from_fitfile).values()]
    else:
        all_performances = [fitness for fitness in get_bin_performances_uniquearchive(path,from_fitfile).values()]
    return all_performances

def global_performances(BD_directory, runs, archive_file_path, max_performance,conversion_func,from_fitfile=False):
    stats = []
    for run in runs:
        stats.append(_global_performance(BD_directory, run, archive_file_path, max_performance,conversion_func,from_fitfile))
    print("global performances: " + str(stats))
    return stats

def _global_performance(BD_directory, run,archive_file_path,max_performance,conversion_func=None,from_fitfile=False):
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
    all_performances = get_all_performances(path,conversion_func,from_fitfile)
    return max(all_performances)/max_performance


def avg_performances(BD_directory, runs, archive_file_path, max_performance,conversion_func,from_fitfile):
    stats = []
    for run in runs:
        stats.append(_avg_performance(BD_directory, run, archive_file_path, max_performance,conversion_func,from_fitfile))
    print("avg performances: " + str(stats))
    return stats

def _avg_performance(BD_directory, run,archive_file_path,max_performance,conversion_func=None,from_fitfile=False):
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
    all_performances=get_all_performances(path,conversion_func,from_fitfile)
    return np.mean(all_performances)/max_performance

def global_reliabilities(BD_directory,runs,archive_file_path):
    """
    averages the global reliability across the different maps in the combined archive
    :param BD_directory:
    :param runs:
    :param archive_file_path:
    :return:
    """

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

     NOTE: because here we are comparing descriptors with the same algorithm,
     and different descriptors have different meaning for the cells,
     the best performance and filled cells are computed by the different runs of a single setting

     i.e., ignore cells that have not been filled in any of the runs,
     but for cells that are filled compute a map's performance/max(performance) and average it

    :param BD_directory: directory in which all the runs of a BD are located,
            e.g. ~/Desktop/history_obstacles
    :params runs, the number of runs
    :param archive_file_path, the relative path from the BD_directory to the archive file
    :return:
    """
    path = get_archive_filepath(BD_directory, run, archive_file_path)
    all_non_empty_performances = get_bin_performances_uniquearchive(path)

    cell_performances = []

    for bin , archive_perfs in combined_archive.items():  # take the mean across the cells that are not empty across runs
        performance = all_non_empty_performances.get(bin, 0.0)
        max_performance = max(archive_perfs)
        assert performance <= max_performance
        if performance == 0:
            cell_performances.append(0.0)
        else:
            cell_performances.append(performance/max_performance)
    mean = np.mean(cell_performances)
    return mean

def precisions(BD_directory,runs,archive_file_path):
    """
    averages the precision of the different maps in the combined archive
    :param BD_directory:
    :param runs:
    :param archive_file_path:
    :return:
    """
    combined_archive=get_combined_archive(BD_directory, runs, archive_file_path)
    stats = []
    for run in runs:
        stats.append(_precision(combined_archive,BD_directory, run, archive_file_path))
    print("precisions"+str(stats))
    return stats
def _precision(combined_archive, BD_directory, run, archive_file_path):

    """
    Same as global reliability, but for each run, the normalized performance
    is averaged only for the cells that were filled by that algo- rithm **in that run**

    i.e., ignore cells not filled in the current run,
    but for cells that are filled compute a map's performance/max(performance) and average it

    :param BD_directory: directory in which all the runs of a BD are located,
            e.g. ~/Desktop/history_obstacles
    :params runs, the number of runs
    :param archive_file_path, the relative path from the BD_directory to the archive file
    :return:
    """
    path = get_archive_filepath(BD_directory, run, archive_file_path)
    all_non_empty_performances = get_bin_performances_uniquearchive(path)

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
    all_non_empty_performances = get_bin_performances_uniquearchive(path)
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
    all_non_empty_performances = get_bin_performances_uniquearchive(path)
    num_filled=len(all_non_empty_performances)
    return float(num_filled)

def globalcoverage(BD_directory,runs,archive_file_path):
    """
    averages the precision of the different maps in the combined archive
    :param BD_directory:
    :param runs:
    :param archive_file_path:
    :return:
    """
    combined_archive=get_combined_archive(BD_directory, runs, archive_file_path)
    num_filled = len(combined_archive)
    return float(num_filled)
def translated_coverages(t,BD_dir,runs, targets):
    d={target:[] for target in targets}
    relative={target:[] for target in targets}
    for run in runs:
        for target, shape in targets.items():
            archive_file = "analysis" + str(t) + "_" + target + "REDUCED.dat"
            cov = _absolutecoverage(shape, BD_dir, run, archive_file)
            d[target].append(cov)
    print("translated coverages " + str(d))
    return d

def add_boxplotlike_data(stats, y_bottom,y_mid,y_top, y_label,method_index,statistic="mean_SD"):
    """
    return x.25,x.50 and x.75
    :return:
    """
    if statistic!="mean_SD":
        x1,x2,x3=tuple(np.quantile(stats, q=[.25, .50, .75]))
    else:
        sd = np.std(stats)
        x2 = np.mean(stats)
        x1 = x2 - sd
        x3 = x2 + sd
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
    solutions, indexes = get_best_individuals(BD_dir, range(1,6), "archive_"+str(generation)+".dat",number,criterion="fitness")
    with open(outfile+"fitness.txt", 'w') as f:
        i=0
        for key,value in solutions.items():
            f.write("%s %s %.3f \n "%(indexes[i], str(key),value[0]))
            i+=1
        f.flush()



    solutions, indexes = get_best_individuals(BD_dir, range(1, 6), "archive_"+str(generation)+".dat", number, criterion="diversity")
    with open(outfile + "diversity.txt", 'w') as f:
        i = 0
        for array in solutions:
            f.write("%s %s %.3f \n" % (indexes[i], array[0:-1], array[-1]))
            i += 1

def try_add_performance_data(i,bd_shapes,directory,runs,archive_file, y_bottom,y_mid,y_top,from_fitfile=False):
    try:
        avg_perform = avg_performances(directory, runs, archive_file, 1.0,
                                       conversion_func=None,from_fitfile=from_fitfile)
        add_boxplotlike_data(avg_perform, y_bottom, y_mid, y_top, y_label="average_performance", method_index=i)

        global_perform = global_performances(directory, runs, archive_file, 1.0,
                                             conversion_func=None,from_fitfile=from_fitfile)
        add_boxplotlike_data(global_perform, y_bottom, y_mid, y_top, y_label="global_performance", method_index=i)

        if not from_fitfile:
            # coverage = coverages(bd_shapes[i], directory, runs, archive_file)
            # add_boxplotlike_data(coverage, y_bottom, y_mid, y_top, y_label="coverage", method_index=i)

            absolutecoverage = absolutecoverages(bd_shapes[i], directory, runs, archive_file)
            add_boxplotlike_data(absolutecoverage, y_bottom, y_mid, y_top, y_label="absolute_coverage", method_index=i)

            globalcov = globalcoverage(directory, runs, archive_file)
            add_boxplotlike_data([globalcov], y_bottom, y_mid, y_top, y_label="global_coverage", method_index=i)

            global_reliability = global_reliabilities(directory, runs, archive_file)
            add_boxplotlike_data(global_reliability, y_bottom, y_mid, y_top, y_label="global_reliability", method_index=i)
    except Exception as e:
        print(e)


def development_plots(title,runs,times,BD_directory,title_tag, fig=None,ax=None):

    # bd_type = ["history","cvt_mutualinfo","cvt_mutualinfoact","cvt_spirit"]  #legend label
    #
    # legend_labels=["handcrafted","mutualinfo","mutualinfoact","spirit"]  # labels for the legend
    # colors=["C"+str(i) for i in range(len(bd_type))]  # colors for the lines
    # # (numsides, style, angle)
    # markers=[(3,1,0), (3,2,0),(4,1,0),(4,2,0)] # markers for the lines
    # bd_shapes = [32**2, 1000,1000,1000]  # shape of the characterisation
    # y_labels=["global_performance","global_reliability","precision","coverage"]

    # bd_type = ["baseline","history","cvt_rab_spirit","Gomes_sdbc_walls_and_robots_std","environment_diversity","environment_diversity"]  #legend label
    # legend_labels=["design","handcrafted","SPIRIT","SDBC","QED","QED-Translated"]  # labels for the legend

    bd_type = ["history","Gomes_sdbc_walls_and_robots_std","cvt_rab_spirit","environment_diversity","environment_diversity","baseline"]  #legend label
    legend_labels=["handcrafted","SDBC","SPIRIT","QED","QED-transfer","baseline"]  # labels for the legend

    colors=["C0","C1","C2","C3","C3","C4"]  # colors for the lines
    # (numsides, style, angle)
    markers=[(1,1,0),(1,2,0),(1,3,0),(3,1,0),(3,2,0),(3,3,0),(4,1,0),(4,2,0),(4,3,0)] # markers for the lines
    bd_shapes =[4096, 4096, 4096,4096, 4096, 4096,4096,4096, 4096]  # shape of the characterisation
    y_labels=["global_performance","average_performance","absolute_coverage","global_coverage","global_reliability"]


    boxes=[(.10,.40),(.10,.60),(.10,.60),(.45,.15),(0.20,0.20),(0.20,0.20)] # where to place the legend box
    y_bottom={ylabel:[[] for i in bd_type] for ylabel in y_labels}
    y_mid = {ylabel: [[] for i in bd_type]  for ylabel in y_labels}
    y_top = {ylabel: [[] for i in bd_type]  for ylabel in y_labels}


    for time in times:
        for i in range(len(bd_type) - 1):
            translated = "->" in legend_labels[i]
            transfered = legend_labels[i].endswith("transfer")
            if translated:
                # continue
                # archive_file="analysis"+str(time)+"_handcrafted.dat"  # just use the one that is quickest
                directory = BD_directory+"/"+bd_type[i] + "/FAULT_NONE"
                if "handcrafted" in legend_labels[i]:
                    archive_file = "analysis" + str(time) + "_handcraftedREDUCED.dat"
                elif "SDBC" in legend_labels[i]:
                    archive_file = "analysis" + str(time) + "_sdbcREDUCED.dat"
                elif "SPIRIT" in legend_labels[i]:
                    archive_file = "analysis" + str(time) + "_spiritREDUCED.dat"
                else:
                    raise Exception("")
            elif transfered:
                recorded_time=5000
                directory = BD_directory+"/"+bd_type[i] + "/FAULT_NONE"
                archive_file = "analysis" + str(recorded_time) + "_handcraftedREDUCED.dat"
            else:
                archive_file="archive_" + str(time) + ".dat"
                directory = BD_directory + "/" + bd_type[i]


            #abs_coverage=absolutecoverages(bd_shapes[i], directory, runs, archive_file)
            #add_boxplotlike_data(abs_coverage, y_bottom, y_mid, y_top, y_label="absolute_coverage",method_index=i)
            try_add_performance_data(i,bd_shapes,directory,runs,archive_file, y_bottom,y_mid,y_top,from_fitfile=translated or transfered)
        # now add baseline
        i = len(bd_type) - 1
        directory = BD_directory + "/" + bd_type[i] + "/"
        archive_file = "fitness"
        try_add_performance_data(i, bd_shapes, directory, runs, archive_file, y_bottom, y_mid, y_top,
                                 from_fitfile=True)
            #precision=precisions(directory, runs, archive_file)
            #add_boxplotlike_data(precision, y_bottom, y_mid, y_top, y_label="precision",method_index=i)


    j=0

    for label in y_labels:
        ylim=[0,4096] if label in ["absolute_coverage","global_coverage"]   else [0.0,1.0]
        axis = None if ax is None else ax[j]
        temp_labels = copy.copy(legend_labels)
        if not label.endswith("performance"):
            #strip baseline and transfer
            temp_labels.remove("baseline")
            temp_labels.remove("QED-transfer")

        # createPlot(y_mid[label],x_values=np.array(times),colors=colors,markers=markers,xlabel="generations",ylabel=label.replace("_"," "),ylim=ylim,
        #            save_filename=RESULTSFOLDER+"/"+title_tag+label+".pdf",legend_labels=legend_labels,
        #            xlim=None,xscale="linear",yscale="linear",
        #        legendbox=boxes[j],annotations=[],xticks=[],yticks=[],task_markers=[],scatter=False,
        #        legend_cols=1,legend_fontsize=26,legend_indexes=[],additional_lines=[],index_x=[],
        #        xaxis_style="plain",y_err=[],force=True,fill_between=(y_bottom[label],y_top[label]))
        createPlot(y_mid[label],x_values=np.array(times),colors=colors,markers=markers,xlabel="generations",ylabel=label.replace("_"," "),ylim=ylim,
                   save_filename=RESULTSFOLDER+"/"+title_tag+label+".pdf",legend_labels=temp_labels,
                   xlim=[0,10500],xscale="linear",yscale="linear",
               legendbox=boxes[j],annotations=[],xticks=[],yticks=[],task_markers=[],scatter=False,
               legend_cols=1,legend_fontsize=26,legend_indexes=[],additional_lines=[],index_x=[],
               xaxis_style="plain",y_err=[],force=True,fill_between=(y_bottom[label],y_top[label]),
                   ax=axis,title=title )
        j+=1

    try:
        time_index=times.index(5000) #only last
        tl_cv = []
        relative_tl_cv=[]
        with open("coverage_table","w") as f:
            targets = {"handcrafted": 4096, "sdbc": 4096, "spirit": 4096}
            labels = ["handcrafted","SDBC","SPIRIT"]
            f.write("   & \rightarrow %s & \rightarrow %s & \rightarrow %s \\ \n"%(labels[0],labels[1],labels[2]))
            for i in range(len(bd_type) - 1):

                numbers=np.array(translated_coverages(times[time_index], BD_dir + "/environment_diversity/FAULT_NONE", runs,
                                     targets=targets))
                tl_cv.append(numbers)
                base_coverage = float(y_mid["absolute_coverage"][i][time_index])
                relative_tl_cv.append(numbers/base_coverage)  # % of solutions maintained
                f.write(legend_labels[i])
                for cov in relative_tl_cv:
                    f.write("& %d "%(cov))
                f.write("\\\n ")
    except Exception as e:
        print(e)


if __name__ == "__main__":
    
    
    runs=5

    fitfuns= ["Aggregation","Dispersion","Flocking","DecayCoverage","DecayBorderCoverage"] #,"DecayBorderCoverage","Flocking"]
    fig, axs = plt.subplots(5, 5,figsize=(50,40))  # coverage, avg perf., global perf., global reliability
    i=0
    for fitfun in fitfuns:
        data_dir = HOME_DIR + "/Data/ExperimentData"
        title=fitfun+"range0.11"
        # print_best_individuals(
        #     BD_dir="/home/david/Data/ExperimentData/"+fitfun+"range11/Gomes_sdbc_walls_and_robots_std",
        #     outfile="best_solutions_"+fitfun+"NOCORRECT", number=10, generation=1200)
        BD_dir = data_dir + "/"+title

        development_plots(title=fitfun,runs=range(1,3), times=range(0,10500, 500),
                          BD_directory=BD_dir,title_tag="FinalBDComp/"+fitfun+"NOCORRECT",
                          ax = axs[:,i])

        i+=1

    finish_fig(fig, RESULTSFOLDER +"/FinalBDComp/"+fitfun+"ALL.pdf")



    # for fitfun in fitfuns:
    #     data_dir = HOME_DIR + "/DataFinal/coll"
    #     title=fitfun+"range11"
    #     print_best_individuals(
    #         BD_dir="/home/david/DataFinal/coll/"+fitfun+"range11/Gomes_sdbc_walls_and_robots_std",
    #         outfile="best_solutions_"+fitfun+"COLLISIONSTOP", number=10, generation=1200)
    #     development_plots(runs=range(1,6), times=range(0,1250, 50), BD_directory=data_dir + "/"+title,title_tag=fitfun+"COLLISIONSTOP",):q
