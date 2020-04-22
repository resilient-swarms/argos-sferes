
from process_archive_data import *
from foraging_params import *

from plots import *

def plot_avg_data():
    y1=[]
    y2=[]
    y3=[]
    t = range(0, 24000, 4000)
    for bd in bd_type:
        lines1=[]
        lines2=[]
        lines3=[]
        for g in t:
            t1 = []
            t2 = []
            t3 = []
            for run in runs:
                path=datadir+"/Foraging/"+bd+"/results"+str(run)+"/performance_recording"+str(g)+"/foraging_stats_archive.txt"
                data=read_spacedelimited(path)
                for line in data:
                    not_harvesting_times,steps_without_food,nest_without_food = line
                    print()
                    t1.append(float(not_harvesting_times)/TICKS_PER_SECOND)
                    t2.append(float(steps_without_food)/TICKS_PER_SECOND)
                    t3.append(float(nest_without_food)/TICKS_PER_SECOND)
            lines1.append(np.mean(t1))
            lines2.append(np.mean(t2))
            lines3.append(np.mean(t3))
        y1.append(lines1)
        y2.append(lines2)
        y3.append(lines3)

    createPlot(y1, np.array(t), colors, markers, xlabel="Generations", ylabel="Time lost on food source ($s$)",
               xlim=[0,22000], ylim=[0,20], save_filename="results/time_food.pdf",
               legend_labels=legend_labels, scatter=False, force=True,
               ax=None, title="")
    createPlot(y2, np.array(t), colors, markers, xlabel="Generations", ylabel="Time without holding food ($s$)",
               xlim=[0, 22000], ylim=[0,120], save_filename="results/time_notholding.pdf",
               legend_labels=legend_labels, scatter=False, force=True,
               ax=None, title="")
    createPlot(y3, np.array(t), colors, markers, xlabel="Generations", ylabel="Time lost in nest ($s$)",
               xlim=[0, 22000], ylim=[0,120], save_filename="results/time_nest.pdf",
               legend_labels=legend_labels, scatter=False, force=True,
               ax=None, title="")
def plot_best_data():
    y1=[]
    y2=[]
    y3=[]
    t = range(0, 24000, 4000)
    for bd in bd_type:
        lines1=[]
        lines2=[]
        lines3=[]
        for g in t:
            t1 = []
            t2 = []
            t3 = []
            for run in runs:
                path = datadir + "/Foraging/" + bd + "/results" + str(run) + "/performance_recording" + str(g) + "/analysis"+str(g)+"_handcrafted.dat"
                parsed_file_list = read_spacedelimited(path)
                maxind=None
                best_performance=-float("inf")
                for index, item in enumerate(parsed_file_list):
                    performance = float(item[-1])
                    if performance > best_performance:
                        maxind = index
                        best_performance = performance
                path=datadir+"/Foraging/"+bd+"/results"+str(run)+"/performance_recording"+str(g)+"/foraging_stats_archive.txt"
                data=read_spacedelimited(path)
                i=0
                for line in data:
                    if i == maxind:
                        not_harvesting_times,steps_without_food,nest_without_food = line
                        print()
                        t1=float(not_harvesting_times)/TICKS_PER_SECOND
                        t2=float(steps_without_food)/TICKS_PER_SECOND
                        t3=float(nest_without_food)/TICKS_PER_SECOND
                        break
                    i+=1
            lines1.append(t1)
            lines2.append(t2)
            lines3.append(t3)
        y1.append(lines1)
        y2.append(lines2)
        y3.append(lines3)

    createPlot(y1, np.array(t), colors, markers, xlabel="Generations", ylabel="Time lost on food source ($s$)",
               xlim=[0,22000], ylim=[0,20], save_filename="results/time_food.pdf",
               legend_labels=legend_labels, scatter=False, force=True,
               ax=None, title="")
    createPlot(y2, np.array(t), colors, markers, xlabel="Generations", ylabel="Time without holding food ($s$)",
               xlim=[0, 22000], ylim=[0,120], save_filename="results/time_notholding.pdf",
               legend_labels=legend_labels, scatter=False, force=True,
               ax=None, title="")
    createPlot(y3, np.array(t), colors, markers, xlabel="Generations", ylabel="Time lost in nest ($s$)",
               xlim=[0, 22000], ylim=[0,120], save_filename="results/time_nest.pdf",
               legend_labels=legend_labels, scatter=False, force=True,
               ax=None, title="")

if __name__ == "__main__":
    plot_best_data()