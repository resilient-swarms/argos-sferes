import csv
import numpy as np
import sys, os
from itertools import product

import argparse

import argparse

import matplotlib as mpl

import sys,os
HOME_DIR = os.environ["HOME"]


parser = argparse.ArgumentParser(description='Process arguments.')
parser.add_argument('-c', type=str,help='command to perform (required)')
parser.add_argument('-p', type=str, help="archive path where the individuals are located" )
parser.add_argument('-o', type=str, help="outputfolder" )
args = parser.parse_args()


def read_spacedelimited(path):
    with open(path) as f:
        reader = csv.reader(f, delimiter=" ")

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
        individuals, indexes = get_combined_archive(BD_directory, runs, archive_file_path, by_bin=True,
                                                    include_val=True, include_ind=True)
        return get_best_fitness_individuals(individuals,number),indexes
    else:
        individuals, indexes = get_combined_archive(BD_directory, runs, archive_file_path, by_bin=False,
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
    os.system("cd "+ outputfolder + " && GZIP=-9 tar cvzf "+file+".tar.gz "+file+"  && rm "+file)
def run_individuals(command, path, outputfolder):

    individuals = get_individuals(path)    
    for individual in individuals:
        new_command = command + " -n "+individual
        print(" performing command :"+str(new_command))
        os.system(new_command)

        # prevent file too big files
        for analysis_suffix in ["sa_history","xy_history"]:
            compress_and_remove(outputfolder, analysis_suffix+str(individual))


def get_bin_performances(path,as_string=True, add_indiv=False):
    parsed_file_list=read_spacedelimited(path)
    bin_performance_dict={}
    individuals=[]
    for item in parsed_file_list:
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
        return bin_performance_dict, individuals

def get_archive_filepath(BD_directory,run, archive_file_path):
    return BD_directory + "/results" + str(run) + "/" + archive_file_path


def get_combined_archive(BD_directory,runs, archive_file_path,by_bin=True,include_val=True,include_ind=False):
    if by_bin:
        combined_archive={}
    else:
        combined_archive=[]
    if include_ind:
        individuals = []
    for run in runs:
        filepath=get_archive_filepath(BD_directory, run, archive_file_path)
        if include_ind:

            bin_performance_dict,indiv=get_bin_performances(filepath,as_string=by_bin, add_indiv=True)
            for ind in indiv:
                individuals.append(ind)
        else:
            bin_performance_dict = get_bin_performances(filepath, as_string=by_bin)
        for key, value in bin_performance_dict.items():
            if by_bin:
                if key in combined_archive:
                    combined_archive[key].append(value)
                else:
                    combined_archive[key]=[value]
            else:
                if include_val:
                    a = np.array(key+(value,),dtype=float)
                else:
                    a = np.array(key, dtype=float)
                combined_archive.append(a)

    if include_ind:
        return combined_archive, individuals
    return combined_archive


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


if __name__ == "__main__":
    run_individuals(args.c, args.p, args.o)
