import csv
import numpy as np
import sys, os
from itertools import product

import argparse

import argparse
from collections import OrderedDict

from distance_metrics import *


import matplotlib as mpl

import sys,os
HOME_DIR = os.environ["HOME"]


parser = argparse.ArgumentParser(description='Process arguments.')
parser.add_argument('-c', type=str,help='command to perform (required)')
parser.add_argument('-p', type=str, help="archive path where the individuals are located" )
parser.add_argument('-o', type=str, help="outputfolder" )
parser.add_argument('-b', type=str, help="best or not")
parser.add_argument('-g', type=str, help="generation")
args = parser.parse_args()


def read_spacedelimited(path):
    with open(path) as f:
        reader = csv.reader(f, delimiter=" ")

        d = list(reader)
        for i in range(len(d)):
            temp=[item for item in d[i] if item != ""]  # remove empty strings
            d[i]=temp
    return d

def read_tabdelimited(path):
    with open(path) as f:
        reader = csv.reader(f, delimiter="\t")

        d = list(reader)
        for i in range(len(d)):
            temp=[item for item in d[i] if item != ""]  # remove empty strings
            d[i]=temp
    return d



def get_individuals(path,as_string=True):
    parsed_file_list=read_spacedelimited(path)
    individuals=[]
    for item in parsed_file_list:
        b=item[0]
        if as_string:
            b=str(b)
        individuals.append(b)
    return individuals

def get_best_individuals(BD_directory, runs, archive_file_path,number, criterion="fitness"):


    if criterion=="fitness":
        individuals, indexes = get_combined_archive(BD_directory, runs, archive_file_path, by_bin="bd",
                                                    include_val=True, include_ind=True)
        return get_best_fitness_individuals(individuals,number),indexes
    else:
        individuals, indexes = get_combined_archive(BD_directory, runs, archive_file_path, by_bin="list",
                                                    include_val=True, include_ind=True)
        return get_best_diversity_individuals(individuals,indexes)

def get_best_fitness_individuals(individuals,number):
    keys = sorted(individuals, key=individuals.get, reverse=True)[:number]
    new_dict = {key: individuals[key] for key in keys}
    return new_dict

def get_best_diversity_individuals(behavs,indivs):

    l = []
    inds = []
    dim = len(behavs[0] - 1)
    new_dict = {}
    for i in range(dim):
        descriptor = [behav[i] for behav in behavs]
        # get the highest value of this descriptor
        j = np.argmax(descriptor)
        l.append(behavs[j])
        inds.append(indivs[j])
        # get the lowest value of this descriptor
        k = np.argmin(descriptor)
        l.append(behavs[k])
        inds.append(indivs[k])
    return l,inds
def compress_and_remove(outputfolder, file):
    os.system("cd "+ outputfolder + " && GZIP=-9 tar cvzf "+file+".tar.gz " +file +" && rm "+file)
def compress_and_remove_lzma(outputfolder, file):
    os.system("cd "+ outputfolder + " && "+lzma_compress(file)+" && rm "+file)


def decompress_lzma(filename):
    directory=os.path.dirname(filename)
    os.system("7z e -mm=LZMA -mx=9 " + filename + ".zip -o"+directory)


def perform_ppm(file, from_zip=False):
    if not from_zip:
        os.system("7z a -mm=PPMd -mmem=256M -mx=9 -mo=32 " + file + ".zip " + file)
    return os.path.getsize(file + ".zip")


def lzma_compress(file):
    return "7z a -mm=LZMA -mx=9 " + file + ".zip " + file



def perform_lzma(file,from_zip=False):
    if not from_zip:
        os.system(lzma_compress(file))
    return os.path.getsize(file + ".zip")

def run_individual(command, individual):
    new_command = command + " -n " + individual
    print(" performing command :" + str(new_command))
    os.system(new_command)

    # prevent file too big files
    # for analysis_suffix in ["sa_history", "xy_history"]:
    #     compress_and_remove(outputfolder, analysis_suffix + str(individual))
def run_individuals(command, path):
    # Parallelizing using Pool.map()
    # import multiprocessing as mp
    #
    # individuals = get_individuals(path)
    # pool = mp.Pool(mp.cpu_count())
    # pool.starmap(run_individual,[(command,i) for i in individuals])
    #
    # pool.close()
    import time
    individuals = get_individuals(path)
    print(individuals)
    time.sleep(5)
    for i in individuals:
        run_individual(command,i)

def run_best_individual(command, outputfolder, generation):
    print("looking for "+outputfolder + "/analysis"+generation+"_handcrafted.dat")
    maxind = get_best_individual(outputfolder + "/analysis"+generation+"_handcrafted.dat")
    print("start run best individual: "+str(maxind))
    run_individual(command, maxind)
    #for analysis_suffix in ["sa_history", "xy_history"]:
    #    compress_and_remove_lzma(outputfolder, analysis_suffix + str(maxind)+".temp")

# def compress_histories(outputfolder):
#     print("looking for " + outputfolder + "/analysis_sdbc.dat")
#     maxind = get_best_individual(outputfolder + "/analysis_sdbc.dat")
#     print("start compress best individual history: " + str(maxind))
#
#     compress_and_remove(outputfolder,outputfolder+"/sa_history"+str(maxind))
#     compress_and_remove(outputfolder, outputfolder + "/xy_history" + str(maxind))


def get_best_individual(path, as_string=False, add_performance=False, add_all=False, index_based=False):
        best_performance=-float("inf")
        maxind=np.nan
        parsed_file_list = read_spacedelimited(path)
        for index, item in enumerate(parsed_file_list):
            indiv = item[0]
            b = tuple(item[1:-1])
            if as_string:
                b = str(b)
            performance = float(item[-1])
            if performance > best_performance:
                maxbd=b
                if index_based:
                    maxind = index
                else:
                    maxind=indiv
                best_performance = performance
        if add_all:
            return maxind, best_performance, maxbd
        if add_performance:
            return maxind, best_performance

        return maxind


def get_baseline_fitness(path):
    parsed_file_list=read_spacedelimited(path)
    for item in parsed_file_list:
        performance = float(item[0])
        return performance

def get_ind_performances_uniquearchive(path):
    parsed_file_list=read_spacedelimited(path)
    bin_performance_dict=OrderedDict()
    for item in parsed_file_list:
        ind = item[0]
        performance=float(item[-1])
        bin_performance_dict[ind]=performance
    return bin_performance_dict
def get_individual_bd(path,ind,bd_start=1):
    parsed_file_list = read_spacedelimited(path)

    for item in parsed_file_list:
            individual = item[bd_start-1]
            if individual==ind:
                return np.array(tuple(item[bd_start:-1]),float)
    raise Exception("individual not found")

def get_individual_bds(path,ind=[],bd_start=1):
    parsed_file_list = read_spacedelimited(path)
    temp=[]
    if ind:
        return [get_individual_bd(path, i, bd_start) for i in ind]
    else:
        for item in parsed_file_list:
            temp.append(np.array(tuple(item[bd_start:-1]),float))
    return temp


def parse_bins(path,bin_index):
    parsed_file_list=read_spacedelimited(path)
    bins=[]
    for item in parsed_file_list:
        b=str(tuple([item[i] for i in bin_index]))
        bins.append(b)
    return bins



def get_bin_performances_uniquearchive(path,as_string=True, add_indiv=False,fitnessfile=False):
    """
    get the bin performance dict assuming the archive includes no duplicate bins
    """
    parsed_file_list=read_spacedelimited(path)
    bin_performance_dict=OrderedDict()
    individuals=[]
    for item in parsed_file_list:
        if not fitnessfile:
            ind = item[0]
            b=tuple(item[1:-1])
            if as_string:
                b=str(b)
        performance=float(item[-1])
        bin_performance_dict[b]=performance

        individuals.append(ind)

    if not add_indiv:
        return bin_performance_dict
    else:
        # check that individuals are correct
        i=0
        print(len(bin_performance_dict))
        print(len(individuals))
        for bin,performance in bin_performance_dict.items():
            b=individuals[i]
            print(i)
            assert b==bin, str(b)+" vs "+str(bin)
            i+=1
        return bin_performance_dict, individuals


def get_bin_performances_duplicatearchive(path,add_function,helper_data,as_string=True,match_individuals=[]):
    """
    get the bin performance dict assuming the archive includes duplicate bins
    """
    parsed_file_list=read_spacedelimited(path)
    bin_performance_dict={}
    for item in parsed_file_list:
        ind = item[0]
        if match_individuals:
            if ind not in match_individuals:  # only add specific individuals
                continue
        b=tuple(item[1:-1])
        if as_string:
            b=str(b)
        performance=float(item[-1])
        new_entry = (performance, ind)
        add_function(b,helper_data,new_entry,bin_performance_dict)


    return bin_performance_dict


def load_centroids(file):
    parsed_file_list=read_spacedelimited(file)
    centroids=[]
    for item in parsed_file_list:
        centroids.append(np.array(tuple(item),dtype=float))
    return centroids

def get_archive_filepath(BD_directory,run, archive_file_path):
    return BD_directory + "/results" + str(run) + "/" + archive_file_path

def get_bd_dir(fitfun, tag="range0.11"):
    data_dir = "/media/david/Elements/Data/"
    title = fitfun + tag
    BD_dir = data_dir + "/" + title
    return BD_dir


def get_combined_archive(BD_directory,runs, archive_file_path,by_bin="bd",include_val=True,include_ind=False,centroids=[]):
    """
    takes different runs, then combines the archives,
    filling cells filled by any of the runs, with a list of all found solutions across runs for each cell)
    :param BD_directory:
    :param runs:
    :param archive_file_path:
    :param by_bin:  "bd", "individual" or "list" (use bd for mapelites, individual for cvtmapelites)
    :param include_val:
    :param include_ind:
    :return:
    """
    if by_bin != "list":
        combined_archive=OrderedDict({})
    else:
        combined_archive=[]
    if include_ind:
        individuals = []
    for run in runs:
        filepath=get_archive_filepath(BD_directory, run, archive_file_path)
        if by_bin=="individual":
            bin_performance_dict=get_ind_performances_uniquearchive(filepath)

        else:
            if include_ind:

                bin_performance_dict,indiv=get_bin_performances_uniquearchive(filepath,as_string=True, add_indiv=True)
                for ind in indiv:
                    individuals.append(ind)
            else:
                bin_performance_dict = get_bin_performances_uniquearchive(filepath, as_string=True)
        for key, value in bin_performance_dict.items():
            if by_bin != "list":
                if key in combined_archive:
                    combined_archive[key].append(value)
                else:
                    combined_archive[key] = [value]
            else:
                if include_val:
                    a = np.array(key + (value,), dtype=float)
                else:
                    a = np.array(key, dtype=float)
                combined_archive.append(a)

    if include_ind:
        return combined_archive, individuals
    return combined_archive

def transform_bd_cvtmapelites(bd,centroids):
    min_dist = float("inf")
    min_index = None
    for i in range(len(centroids)):
        dist = Euclidian_dist(centroids[i], bd)
        if (dist < min_dist):
            min_dist = dist
            min_centr = centroids[i]
            min_index = i
    return min_index,tuple(min_centr)

def combine_archive_CVTMAPElites(combined_archive,key, value,centroids,by_bin, include_val):
    min_index,min_centr = transform_bd_cvtmapelites(np.array(tuple(key),dtype=float), centroids)
    if by_bin:
        if min_index in combined_archive:
            combined_archive[min_index].append(value)
        else:
            combined_archive[min_index] = [value]
    else:
        if include_val:
            a = np.array(min_centr + (value,), dtype=float)
        else:
            a = np.array(min_centr, dtype=float)
        combined_archive.append(a)
def sorting_function(behaviour):
    """
    sorting function for a generic behavioural descriptor
    :param behaviour:
    :return:
    """
    N=len(behaviour)
    s =  sum(behaviour[j]*2**j for j in range(N))
    max = sum(1.0 * 2**j for j in range(N))
    return s/max


def get_labels(N):
    """
    sorting function for a generic behavioural descriptor
    :param behaviour:
    :return:
    """
    return


def frange(start, stop, step):
    i = start
    while i < stop:
        yield str(i)
        i += step
def get_bins(bd_shape):
    if isinstance(bd_shape,tuple):
        bd_dim,bins=bd_shape
        return pow(bins,bd_dim)
    else:
        # assume you are doing cvt , just return the shape=number of centroids
        return bd_shape
def convert_QED_rep(individual):
    s=""
    individual=int(individual)
    for num in range(5,-1,-1):
        number=individual//(4**(num))
        individual-=number*4**num
        s+=str(number+1) + ","
    return s[:-1]

if __name__ == "__main__":
    # sys.path.append("/home/david/DataFinal/ExperimentData")
    # os.system("cd /home/david/DataFinal/ExperimentData")
    # args.c = "bin/analysis6D/Aggregationrange11/environment_diversity/FAULT_NONE/exp_5.argos all 1000 " \
    #          "-d  Aggregationrange11/environment_diversity/FAULT_NONE/results5 " \
    #          "--load  Aggregationrange11/environment_diversity/results5/gen_01000 --o outputfile"
    # args.p = "/home/david/DataFinal/ExperimentData/Aggregationrange11/environment_diversity/results5/archive_1000.dat"
    # args.o= "/home/david/DataFinal/ExperimentData/Aggregationrange11/environment_diversity/FAULT_NONE/results5"
    # args.b = "all"
    if args.b == "best":
        run_best_individual(args.c, args.o, args.g)
    # elif args.b == "compress":
    #     compress_histories(args.o)
    else:
        run_individuals(args.c, args.p)
