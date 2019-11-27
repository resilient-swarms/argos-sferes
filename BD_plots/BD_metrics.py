from plots import *
from dimensionality_plot import *
# behavioural metrics defined in Mouret, J., & Clune, J. (2015). Illuminating search spaces by mapping elites. 1–15.
import os
import operator
HOME_DIR = os.environ["HOME"]
RESULTSFOLDER="results"
import pickle
import copy
from perturbance_metrics import *
import numpy as np
PRINT=False

baseline_performances = pickle.load(open("data/fitfun/maximal_fitness.pkl", "rb"))

def print_conditional(some_string):
    if PRINT:
        print(some_string)

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

    print_conditional("global performances: " + str(stats))
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
    print_conditional("avg performances: " + str(stats))
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

def global_reliabilities(BD_directory,runs,archive_file_path,by_bin):
    """
    averages the global reliability across the different maps in the combined archive
    :param BD_directory:
    :param runs:
    :param archive_file_path:
    :return:
    """

    combined_archive=get_combined_archive(BD_directory, runs, archive_file_path,by_bin=by_bin)
    stats = []
    for run in runs:
        stats.append(_global_reliability(combined_archive,BD_directory, run, archive_file_path,by_bin))
    print_conditional("global reliabilities: "+str(stats))
    return stats

def _global_reliability(combined_archive,BD_directory, run, archive_file_path,by_bin):

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
    if by_bin == "individual":
        all_non_empty_performances = get_ind_performances_uniquearchive(path)

    else:
        all_non_empty_performances = get_bin_performances_uniquearchive(path, as_string=True)

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

def precisions(BD_directory,runs,archive_file_path,by_bin):
    """
    averages the precision of the different maps in the combined archive
    :param BD_directory:
    :param runs:
    :param archive_file_path:
    :return:
    """
    combined_archive=get_combined_archive(BD_directory, runs, archive_file_path,by_bin=by_bin)
    stats = []
    for run in runs:
        stats.append(_precision(combined_archive,BD_directory, run, archive_file_path))
    print_conditional("precisions"+str(stats))
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
    print_conditional("global coverages"+str(stats))
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
    print_conditional("global coverages"+str(stats))
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

def _spread(bd_shape,BD_directory, run, archive_file_path,
            individuals=[],distance_metric=norm_Euclidian_dist,
            bd_start=1,comp=[]):
    """
    Measures how many cells of the feature space a run of an algorithm
    is able to fill
    :return:
    """

    path = get_archive_filepath(BD_directory, run, archive_file_path)
    bd_list  = get_individual_bds(path,individuals,bd_start)
    temp=0.0
    comps=0.0




    if not comp:
        for i,bd1 in enumerate(bd_list):
            for j in range(i+1,len(bd_list)):
                    temp+=distance_metric(bd1,bd_list[j])
                    comps+=1.0
        temp/=comps
        assert comps==len(bd_list)*(len(bd_list)-1)/2
        return temp
    else:
        comp =  get_individual_bds(path,comp,bd_start)[0]


        for i,bd1 in enumerate(bd_list):
            temp+=distance_metric(bd1,comp)  # no need to check for equality since if comp==bd1 this should be penalised
            comps+=1.0
        temp/=comps
        assert comps==len(bd_list)
        return temp

def uniqueness(BD_directory,runs, gener,targets, bin_indexes):
    """
    calculates uniqueness values, for combined archive and single archive
    :param BD_directory:
    :param runs:
    :param archive_file_path:
    :param bin_indexes
    :return:
    """
    ub_dict={}
    comb_ub_dict={}
    for target in targets:
        comb_unique_bins=set([])
        unique_bins=[]
        for run in runs:

            u=set(parse_bins(BD_directory+"/results"+str(run)+"/analysis"+str(gener)+"_"+target+"REDUCED.dat",bin_indexes[target]))
            unique_bins.append(u)
            comb_unique_bins = comb_unique_bins | u
        ub_dict[target]=unique_bins
        comb_ub_dict[target]=comb_unique_bins

    return ub_dict,comb_ub_dict


def globalcoverage(BD_directory,runs,archive_file_path,by_bin):
    """
    averages the precision of the different maps in the combined archive
    :param BD_directory:
    :param runs:
    :param archive_file_path:
    :return:
    """
    combined_archive=get_combined_archive(BD_directory, runs, archive_file_path,by_bin=by_bin)
    num_filled = len(combined_archive)
    return float(num_filled)
def translated_coverages(t,BD_dir,runs, targets):
    d={target:[] for target in targets}
    for run in runs:
        for target, shape in targets.items():
            archive_file = "analysis" + str(t) + "_" + target + "REDUCED.dat"
            cov = _absolutecoverage(shape, BD_dir, run, archive_file)
            d[target].append(cov)
    print_conditional("translated coverages " + str(d))
    return d

def translated_spreads(t,BD_dir,runs,targets,bd_start,dists,individuals,comp):
    d = {target: [] for target in targets}
    for run in runs:
        for target, shape in targets.items():
            if individuals[run-1]:  # look for the unreduced archive
                archive_file = "analysis" + str(t) + "_" + target + ".dat"
            else: # look for the reduced archive individuals
                archive_file = "analysis" + str(t) + "_" + target + "REDUCED.dat"
            s = _spread(shape, BD_dir, run, archive_file,bd_start=bd_start.get(target,1),
                        distance_metric=dists[target],
                        individuals=individuals[run-1],
                        comp=comp[run-1])
            d[target].append(s)
    print_conditional("translated spreads " + str(d))
    return d

def add_boxplotlike_data(stats, y_bottom,y_mid,y_top, y_label,method_index,statistic="mean_SD"):
    """
    return x.25,x.50 and x.75
    :return:
    """
    if statistic=="median_IQR":
        x1,x2,x3=tuple(np.quantile(stats, q=[.25, .50, .75]))
    elif statistic=="mean_SD":
        sd = np.std(stats)
        x2 = np.mean(stats)
        x1 = x2 - sd
        x3 = x2 + sd
    elif statistic=="meanall_replicatesd": # when joining different fitfuns

        x2=np.mean(np.array(stats))
        sds=[np.std(stats[i]) for i in range(len(stats))]
        sd=np.mean(sds)
        x1= x2 - sd
        x3 = x2 + sd
        # assumes fitfun is first dimension of stats

    else:
        raise Exception("statistic %s not known"%(statistic))

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


def try_add_performance_data(i,bd_shapes,bybin_list,directory,runs,archive_file, y_bottom,y_mid,y_top,from_fitfile=False):
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

            globalcov = globalcoverage(directory, runs, archive_file,by_bin=bybin_list[i])
            add_boxplotlike_data([globalcov], y_bottom, y_mid, y_top, y_label="global_coverage", method_index=i)

            global_reliability = global_reliabilities(directory, runs, archive_file,by_bin=bybin_list[i])
            add_boxplotlike_data(global_reliability, y_bottom, y_mid, y_top, y_label="global_reliability", method_index=i)
    except Exception as e:
        print(e)
def get_archiveplusdir(BD_directory,bd_type,i,generation,projected=False):
    translated = "->" in legend_labels[i]

    if translated:
        # continue
        # archive_file="analysis"+str(generation)+"_handcrafted.dat"  # just use the one that is quickest
        directory = BD_directory + "/" + bd_type[i] + "/FAULT_NONE"
        if "handcrafted" in legend_labels[i]:
            archive_file = "analysis" + str(generation) + "_handcraftedREDUCED.dat"
        elif "SDBC" in legend_labels[i]:
            archive_file = "analysis" + str(generation) + "_sdbcREDUCED.dat"
        elif "SPIRIT" in legend_labels[i]:
            archive_file = "analysis" + str(generation) + "_spiritREDUCED.dat"
        else:
            raise Exception("")
    elif projected:
        recorded_generation = 30000
        directory = BD_directory + "/" + bd_type[i] + "/FAULT_NONE"
        archive_file = "analysis" + str(recorded_generation) + "_handcraftedREDUCED.dat"
    else:
        archive_file = "archive_" + str(generation) + ".dat"
        directory = BD_directory + "/" + bd_type[i]
    return archive_file,directory


def coverage_development_plots(title,runs,times,BD_directory,title_tag, bd_type, legend_labels,bybin_list,fig=None,ax=None,metrics=None):

    colors=["C0","C1","C2","C3","C3","C4"]  # colors for the lines
    # (numsides, style, angle)
    markers=[(3,1,0),(3,2,0),(4,1,0),(4,2,0)] # markers for the lines
    bd_shapes =[4096, 4096, 4096,4096, 4096, 4096,4096,4096, 4096]  # shape of the characterisation
    y_labels=["absolute_coverage"]


    boxes=[(.10,.40),(.10,.60),(.10,.60),(.45,.15),(0.20,0.20),(0.20,0.20)] # where to place the legend box
    y_bottom={ylabel:[[] for i in bd_type] for ylabel in y_labels}
    y_mid = {ylabel: [[] for i in bd_type]  for ylabel in y_labels}
    y_top = {ylabel: [[] for i in bd_type]  for ylabel in y_labels}



    for time in times:
        for i in range(len(bd_type)):
            print(legend_labels[i])
            projected=legend_labels[i].endswith("projected")
            abs_coverages=[]
            for j in range(len(fitfuns)):
                try:
                    archive_file, directory = get_archiveplusdir(BD_directory[j],bd_type,i,time,projected=projected)
                    abs_coverage=absolutecoverages(bd_shapes[i], directory, runs, archive_file)
                    abs_coverages.append(abs_coverage)

                except Exception as e:
                    print(e)
            add_boxplotlike_data(abs_coverages, y_bottom, y_mid, y_top, y_label="absolute_coverage",method_index=i,
                                 statistic="meanall_replicatesd")


    j=0
    maximum_line = (times,[4096 for i in times])
    cov=y_mid["absolute_coverage"]
    top_cov=y_top["absolute_coverage"]
    for i in range(len(cov)):
        print(bd_type[i])
        print(cov[i][-1])
        sd_cov = top_cov[i][-1] - cov[i][-1]
        print(sd_cov)
    annots = {"text": "maximal coverage=4096","xy":(5000,4400),"xytext":(5000,4400),
              "fontsize":22,"align": "center"}
    for label in y_labels:
        #ylim=[0,4500]

        createPlot(y_mid[label],x_values=np.array(times),
                   save_filename=RESULTSFOLDER + "/" + title_tag + label + ".pdf", legend_labels=legend_labels,
                   colors=colors,markers=markers,xlabel="generations",ylabel=label.replace("_"," "),
                   xlim=[0,times[-1]+500],xscale="linear",yscale="log",ylim=[10**1,10**4],
                   legendbox=boxes[j],annotations=[annots],xticks=[],yticks=[],task_markers=[],scatter=False,
                   legend_cols=1,legend_fontsize=26,legend_indexes=[],additional_lines=[maximum_line],index_x=[],
                   xaxis_style="plain",y_err=[],force=True,fill_between=(y_bottom[label],y_top[label]),
                   ax=ax,title=title )

        j+=1


def development_plots(title,runs,times,BD_directory,title_tag, bd_type, legend_labels,bybin_list,fig=None,ax=None,metrics=None):

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
        for i in range(len(bd_type)):
            projected=legend_labels[i].endswith("projected")
            baseline=legend_labels[i]=="baseline"
            print(legend_labels[i])
            archive_file, directory = get_archiveplusdir(BD_directory,bd_type,i,time,projected=projected)
            #abs_coverage=absolutecoverages(bd_shapes[i], directory, runs, archive_file)
            #add_boxplotlike_data(abs_coverage, y_bottom, y_mid, y_top, y_label="absolute_coverage",method_index=i)
            try_add_performance_data(i,bd_shapes,bybin_list,directory,runs,archive_file, y_bottom,y_mid,y_top,from_fitfile=projected or baseline)
        # now add baseline
        # i = len(bd_type) - 1
        # directory = BD_directory + "/" + bd_type[i] + "/"
        # archive_file = "fitness"
        # try_add_performance_data(i, bd_shapes, bybin_list,directory, runs, archive_file, y_bottom, y_mid, y_top,
        #                          from_fitfile=True)
            #precision=precisions(directory, runs, archive_file)
            #add_boxplotlike_data(precision, y_bottom, y_mid, y_top, y_label="precision",method_index=i)


    j=0

    for label in y_labels:
        ylim=[0,4096] if label in ["absolute_coverage","global_coverage"]   else [0.0,1.0]
        axis = None if ax is None else ax[j]
        temp_labels = copy.copy(legend_labels)
        #if not label.endswith("performance"):
            #strip baseline and transfer
            #temp_labels.remove("baseline")
            #temp_labels.remove("QED-projected")

        # createPlot(y_mid[label],x_values=np.array(times),colors=colors,markers=markers,xlabel="generations",ylabel=label.replace("_"," "),ylim=ylim,
        #            save_filename=RESULTSFOLDER+"/"+title_tag+label+".pdf",legend_labels=legend_labels,
        #            xlim=None,xscale="linear",yscale="linear",
        #        legendbox=boxes[j],annotations=[],xticks=[],yticks=[],task_markers=[],scatter=False,
        #        legend_cols=1,legend_fontsize=26,legend_indexes=[],additional_lines=[],index_x=[],
        #        xaxis_style="plain",y_err=[],force=True,fill_between=(y_bottom[label],y_top[label]))
        createPlot(y_mid[label],x_values=np.array(times),colors=colors,markers=markers,xlabel="generations",ylabel=label.replace("_"," "),ylim=None,
                   save_filename=RESULTSFOLDER+"/"+title_tag+label+".pdf",legend_labels=temp_labels,
                   xlim=[0,times[-1]+500],xscale="linear",yscale="linear",
               legendbox=boxes[j],annotations=[],xticks=[],yticks=[],task_markers=[],scatter=False,
               legend_cols=1,legend_fontsize=26,legend_indexes=[],additional_lines=[],index_x=[],
               xaxis_style="plain",y_err=[],force=True,fill_between=(y_bottom[label],y_top[label]),
                   ax=axis,title=title )
        j+=1

def get_all_best_individuals(BD_dir,runs,faults,gen,types=None):
    """
    get all the best individuals: FAULT_NONE, run1_p0..p39, run2_p0..p39, etc, and make a unique list of them
    :param BD_dir:
    :return:
    """
    all=[]
    for run in runs:
        if types!="only_fault":
            maxind = get_best_individual(BD_dir+"/FAULT_NONE/results"+str(run)+"/analysis"+str(gen)+"_handcrafted.dat",
                                                      as_string=False, add_performance=False, add_all=False, index_based=False)
            all.append(maxind)

        if types!="only_comp":
            for fault in faults:
                maxind = get_best_individual(
                    BD_dir + "/faultyrun" + str(run) + "_p"+str(fault)+"/results"+str(run)+"/analysis" + str(gen) + "_handcrafted.dat",
                    as_string=False, add_performance=False, add_all=False, index_based=False)

                all.append(maxind)

    return all

def get_spread(source, directory, gener, targets,bd_starts,dists):
    if source == "all":
        return translated_spreads(gener, directory  + "/FAULT_NONE",
                                   runs,
                                   targets=targets,
                                   bd_start=bd_starts,  # bd_start is only required when using the reduced archive
                                   dists=dists,
                                   individuals=[[] for i in runs],
                                   comp=[[] for i in runs]
                                  )
    elif source == "best_comp":
        individuals = [
            get_all_best_individuals(directory , [run], faults, gener, types="only_fault")
            for run
            in runs]
        comp = [
            get_all_best_individuals(directory , [run], faults, gener, types="only_comp") for
            run
            in runs]
        return  translated_spreads(gener, directory + "/FAULT_NONE",
                                   runs,
                                   targets=targets,
                                   bd_start={}, # bd_start is only required when using the reduced archive
                                   dists=dists,
                                   individuals=individuals,
                                   comp=comp)
    else:
        individuals = [get_all_best_individuals(directory, [run], faults, 30000) for run in
                       runs]
        return translated_spreads(gener, directory + "/FAULT_NONE",
                                   runs,
                                   targets=targets,
                                   bd_start={}, # bd_start is only required when using the reduced archive
                                   dists=dists,
                                   individuals=individuals,
                                  comp=[[] for i in runs])


def apply_star_and_bold(text,descriptor,target,max_descriptor,second_max_descriptor):
    if descriptor==max_descriptor:
        text=text+r"^{*}"
        if target == descriptor:
            text=r"$"+text+r"$"
        else:
            text=r"$\mathbf{"+text+"}$"
    else:
        if descriptor==second_max_descriptor:
            text = r"$\mathbf{" + text + "}$"
    return text

def make_translation_table(tab_label,BD_dirs,runs,times,source="all"):


        time_index = times.index(30000)  # only last
        gener=times[time_index]
        with open("results/evolution/table/coverage_table" + tab_label+source, "w") as f:
            targets = OrderedDict({"spirit": 4096})
            labels = ["HBD", "SDBC", "SPIRIT"]
            bd_starts = {"handcrafted":1,"sdbc":2,"spirit":2}
            bin_indexes = {"handcrafted":range(1,4),"sdbc":[1],"spirit":[1]}

            def mv(p1,p2):
                return avg_variation_distance(p1,p2,16)

            dists = {"handcrafted": norm_Euclidian_dist, "sdbc": norm_Euclidian_dist, "spirit": mv}
            f.write(r"   & \multicolumn{4}{c}{$\rightarrow$ %s} & \multicolumn{4}{c}{$\rightarrow$ %s} & \multicolumn{4}{c}{$\rightarrow$ %s} \\ " % (
                labels[0], labels[1], labels[2]))
            f.write("\n")
            #f.write(r"   & coverage & spread & coverage & spread & coverage & spread \\ ")
            f.write("\n")


            combo_u=[]
            for i in range(len(bd_type)):
                combo_u.append({target: set([]) for target in targets})
                for directory in BD_dirs:
                    dirdir = directory + "/" + str(bd_type[i])
                    u,combined_u= uniqueness(dirdir + "/FAULT_NONE", runs, gener, targets, bin_indexes)
                    combo_u[i]={target: combined_u[target] | combo_u[i][target] for target in targets}

            print("computing uniqueness scores")
            for target in targets:
                for i in range(len(bd_type)):
                    print(bd_type[i] + " : target ->"+str(target))
                    unique_score=0
                    for j in range(len(bd_type)):
                        if i!=j:
                            # count the number of elements not contained
                            difference = combo_u[i][target] - combo_u[j][target]
                            unique_score+=len(difference)
                    unique_score/=float(len(bd_type) - 1)
                    print(unique_score)


            for i in range(len(bd_type)):
                print(bd_type[i])
                numbers = {target: [] for target in targets} # gather all the translated coverages for all targets
                numbers2 = {target: [] for target in targets} # gather all the translated spreads for all targets
                numbers3 = {target: [] for target in targets}  # gather all the translated spreads for all targets
                numbers4 = {target: [] for target in targets}  # gather all the translated spreads for all targets
                numbers5 = {target: [] for target in targets}
                for directory in BD_dirs:
                    dirdir=directory + "/" + str(bd_type[i])
                    temp = translated_coverages(gener, dirdir + "/FAULT_NONE",
                                               runs,
                                               targets=targets)
                    print("finished 1")
                    temp2 = get_spread("all",dirdir,gener,targets,bd_starts,dists)
                    print("finished 2")
                    temp3 = get_spread("best", dirdir, gener, targets, bd_starts, dists)
                    print("finished 3")
                    temp4 = get_spread("best_comp", dirdir, gener, targets, bd_starts, dists)
                    print("finished 4")

                    for bd in targets:
                        numbers[bd] = np.append(numbers[bd], temp[bd])
                        numbers2[bd] = np.append(numbers2[bd],temp2[bd])
                        numbers3[bd] = np.append(numbers3[bd], temp3[bd])
                        numbers4[bd] = np.append(numbers4[bd], temp4[bd])
                avg_cov = {bd: np.mean(numbers[bd]) for bd in numbers}
                std_cov = {bd: np.std(numbers[bd]) for bd in numbers}



                avg_spread = {bd: np.mean(numbers2[bd]) for bd in numbers2}
                std_spread = {bd: np.std(numbers2[bd]) for bd in numbers2}


                avg_bspread = {bd: np.mean(numbers3[bd]) for bd in numbers3}
                std_bspread = {bd: np.std(numbers3[bd]) for bd in numbers3}

                avg_bcspread = {bd: np.mean(numbers4[bd]) for bd in numbers4}
                std_bcspread = {bd: np.std(numbers4[bd]) for bd in numbers4}


                # base_coverage = float(y_mid["absolute_coverage"][i][time_index])
                # relative_tl_cv=avg_cov/base_coverage # % of solutions maintained
                f.write(legend_labels[i])
                for tg in targets:
                    f.write(r" & $%d \pm %d$ & $%.3f \pm %.2f$ & $%.3f \pm %.2f$ & $%.3f \pm %.2f$" % (
                    avg_cov[tg], std_cov[tg],
                    avg_spread[tg], std_spread[tg],
                    avg_bspread[tg], std_bspread[tg],
                    avg_bcspread[tg], std_bcspread[tg]))

                    # f.write(r"& %d "%(relcov))
                f.write(r"\\ ")
                f.write("\n")





def make_evolution_table(fitfuns, bd_type, runs, generation,load_existing=False,by_fitfun=True):
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
        import pickle
        if load_existing:
            best_performance_data, coverage_data = pickle.load(open("evolution/evolution_statistics.pkl","rb"))
        else:
            best_performance_data = []
            coverage_data = []
            avg_performance_data = []

            for i in range(len(bd_type)):
                print(bd_type[i])
                best_performance_data.append([])
                avg_performance_data.append([])
                coverage_data.append([])


                for j in range(len(fitfuns)):
                    print(fitfuns[j])
                    BD_dir = get_bd_dir(fitfuns[j])
                    archive_file,directory=get_archiveplusdir(BD_dir,bd_type,i,generation,projected=bd_type[i]=="environment_diversity")
                    # get all the data from the archive: no fault
                    p=global_performances(directory,runs,archive_file,max_performance=1,conversion_func=None)/ baseline_performances[fitfuns[j]]

                    archive_file, directory = get_archiveplusdir(BD_dir, bd_type, i, generation,
                                                                 projected=False)
                    c=coverages(4096,directory,runs, archive_file)

                    a_p = avg_performances(directory,runs,archive_file,max_performance=1,conversion_func=None,from_fitfile=False)/ baseline_performances[fitfuns[j]]
                    if by_fitfun:
                        best_performance_data[i].append(p)
                        avg_performance_data[i].append(a_p)
                        coverage_data[i].append(c)
                    else:
                        best_performance_data[i] = np.append(best_performance_data[i],p)
                        avg_performance_data[i] = np.append(avg_performance_data[i], a_p)
                        coverage_data[i] = np.append(coverage_data[i], c)
        if not load_existing:
            pickle.dump((best_performance_data, avg_performance_data, coverage_data), open("data/evolution_data/evolution_statistics.pkl","wb"))
        from scipy.stats import mannwhitneyu

        with open("results/evolution/table/evolution_table", "w") as f:
            make_table(f, [best_performance_data, coverage_data],
                           rowlabels=fitfuns,
                           columnlabels=legend_labels,
                           conditionalcolumnlabels=[("performance","float2"), ("coverage","float2")],
                       transpose=False)

        with open("results/evolution/table/evolution_table_with_avg", "w") as f:
            make_table(f, [best_performance_data, avg_performance_data,coverage_data],
                           rowlabels=fitfuns,
                           columnlabels=legend_labels,
                           conditionalcolumnlabels=[("best perf.","float2"), ("average perf.","float2"),("coverage","float2")],
                       transpose=False)

        # make_boxplots(best_performance_data, row_conditions=legend_labels,
        #               save_filename="boxplot.png", xlabs=[], ylab="best performance", ylim=[0,1])
        fig, axes = plt.subplots(1, 1, figsize=(10, 10))  # compare fault_none to perturbations
        plt.boxplot(best_performance_data, positions=np.array(range(len(best_performance_data))) * 2.0, sym='', widths=0.6)
        plt.xticks(np.array(range(len(best_performance_data))) * 2.0, legend_labels,fontsize=26)
        axes.tick_params(axis='both', which='major', labelsize=20)
        axes.tick_params(axis='both', which='minor', labelsize=20)
        axes.set_ylim([0.9,1.0])
        plt.ylabel("best performance",fontsize=26)
        plt.tight_layout()
        plt.savefig(RESULTSFOLDER + "/bestperformance_boxplot.pdf")



def create_all_development_plots():
    runs=range(1,6)

    fitfuns= ["Aggregation","Dispersion","DecayCoverage","DecayBorderCoverage","Flocking"] #,"DecayBorderCoverage","Flocking"]
    bd_type = ["history", "Gomes_sdbc_walls_and_robots_std", "cvt_rab_spirit", "environment_diversity"]  # file system label for bd
    legend_labels=["handcrafted","SDBC","SPIRIT","QED"]  # labels for the legend
    bybin_list=["bd", "individual", "individual", "bd"]
    times=range(0,30500, 500)
    fig, axs = plt.subplots(5, 5, figsize=(50, 40))  # coverage, avg perf., global perf., global reliability
    for i,fitfun in enumerate(fitfuns):
        development_plots(title=fitfun,runs=runs, times=times,
                          BD_directory=get_bd_dir(fitfun),title_tag="/"+fitfun+"NOCORRECT",bd_type=bd_type,
                          legend_labels=legend_labels,bybin_list=bybin_list,
                          ax = axs[:,i])


    finish_fig(fig, RESULTSFOLDER +"/evolution/development/All_BD_metrics_allruns.pdf")

def create_coverage_development_plots():
    runs=range(1,6)

    bybin_list=["bd", "individual", "individual", "bd", "bd", ""]
    times=range(0,30500, 500)
    fig, axs = plt.subplots(1,1, figsize=(15, 10))  # coverage, avg perf., global perf., global reliability
    coverage_development_plots(title="",runs=runs, times=times,
                          BD_directory=[get_bd_dir(fitfun) for fitfun in fitfuns],
                         title_tag="",bd_type=bd_type,
                          legend_labels=legend_labels,bybin_list=bybin_list,
                          ax = axs)


    finish_fig(fig, RESULTSFOLDER +"/evolution/development/CoverageLOGSCALE_allruns.pdf")



if __name__ == "__main__":






        # global
    faults=range(50)
    runs=range(1,6)
    fitfuns = ["Aggregation", "DecayCoverage","Dispersion",
               "DecayBorderCoverage","Flocking"]  # ,"DecayBorderCoverage","Flocking"]
    bd_type = ["history","Gomes_sdbc_walls_and_robots_std", "cvt_rab_spirit", "environment_diversity"
             ]  # file system label for bd
    legend_labels = ["HBD", "SDBC", "SPIRIT", "QED"]  # labels for the legend
    generation=30000

    make_translation_table("CORRECT", [get_bd_dir(f) for f in fitfuns], runs,times=[generation],source="best")


    #baseline_performances = pickle.load(open("data/fitfun/maximal_fitness.pkl", "rb"))

    #make_evolution_table(fitfuns, bd_type, runs, generation,load_existing=False,by_fitfun=False)


    #create_coverage_development_plots()

    #create_all_development_plots()