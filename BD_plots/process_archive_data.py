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
parser.add_argument('-c', type=str,
                    help='command to perform (required)')
parser.add_argument('-p', type=str, help="archive path where the individuals are located" )

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

def run_individuals(command, path):

    individuals = get_individuals(path)
    for individual in individuals:
        new_command = command + " -n "+individual
        print(" performing command :"+str(new_command))
        os.system(new_command)




def get_bin_performances(path,as_string=True):
    parsed_file_list=read_spacedelimited(path)
    bin_performance_dict={}
    for item in parsed_file_list:
        b=tuple(item[1:-1])
        if as_string:
            b=str(b)
        performance=float(item[-1])
        bin_performance_dict[b]=performance
    return bin_performance_dict

def get_archive_filepath(BD_directory,run, archive_file_path):
    return BD_directory + "/results" + str(run) + "/" + archive_file_path


def get_combined_archive(BD_directory,runs, archive_file_path,by_bin=True,include_val=True):
    if by_bin:
        combined_archive={}
    else:
        combined_archive=[]
    for run in range(1,runs+1):
        filepath=get_archive_filepath(BD_directory, run, archive_file_path)
        bin_performance_dict=get_bin_performances(filepath,as_string=by_bin)
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
    #bins=get_bins(2, 3)
    #p=get_archives_performances(HOME_DIR+"/argos-sferes/experiments/data/MeanSpeed/history",5,"archive_900.dat")
    run_individuals(args.c, args.p)
