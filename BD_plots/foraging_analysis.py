
from process_archive_data import *
from foraging_params import *

def gather_data():
    for bd in bd_type:
        for run in runs:
            for g in range(0,20000,4000):
                path=datadir+"/Foraging/"+bd+"/results"+str(run)+"/performance_recording"+str(g)+"/foraging_stats_archive.txt"
                data=read_spacedelimited(path)
                for line in data:
                    not_harvesting_times,steps_without_food,nest_without_food = line
                    print()


if __name__ == "__main__":
    gather_data()