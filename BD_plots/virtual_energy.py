from plots import *
from perturbation_analysis_foraging import *
from signature_plot import plot

def gather_data_normal():
    x=[]
    y=[]
    x_full=[]
    for bd in bd_type:
        for run in runs:
            parsed_file_list = read_tabdelimited(HOME_DIR + "/Data/Foraging/"+bd+"/results"+str(run)+"/virtual_energy_exp_init3/performance_recording20000/fitness")
            for item in parsed_file_list:
                f = float(item[0])
                E = float(item[1])
                x.append(f)
                y.append(E)
            parsed_file_list = read_spacedelimited(HOME_DIR + "/Data/Foraging/"+bd+"/results"+str(run)+"/archive_"+str(generation)+".dat")
            for item in parsed_file_list:
                x_full.append(float(item[-1]))
    return x,y,x_full

def gather_data_faulty():
    x = []
    y = []
    x_full = []

    for bd in bd_type:
        for fault in foraging_perturbations:
            print("fault %s" % (fault))
            for r, run in enumerate(runs):
                if fault == "software_foodp3f2" and run == 5:
                    print("skipping")
                    continue
                if fault == "software_foodp4f1" and run == 3:
                    print("skipping")
                    continue
                faultpath = HOME_DIR + "/Data/Foraging/"+bd_type[i] + "/faultyrun" + str(run) + "_" + fault + ""
                parsed_file_list = read_tabdelimited(faultpath)
                for item in parsed_file_list:
                    f = float(item[0])
                    E = float(item[1])
                    x.append(f)
                    y.append(E)
                parsed_file_list = read_spacedelimited(
                    HOME_DIR + "/Data/Foraging/" + bd + "/results" + str(run) + "/archive_" + str(generation) + ".dat")
                for item in parsed_file_list:
                    x_full.append(float(item[-1]))


def create_stats(x,y,x_full):
    # correlation statistics
    print("linear correlation with partial evaluation fitness="+str(np.corrcoef(x,y)))
    coef, p = spearmanr(x, y)
    print("rank correlation with partial evaluation fitness="+str(coef))

    print("linear correlation with full evaluation fitness ="+str(np.corrcoef(x_full,y)))
    coef, p = spearmanr(x_full, y)
    print("rank correlation with full evaluation fitness="+str(coef))


    # trial completion statistics
    percent_full=np.sum(np.array(y)>TICKS_PER_TRIAL)/len(y)
    print("percentage full trial:"+str(percent_full))

    minimal_time=np.min(np.array(y))/TICKS_PER_SECOND
    median_time=np.median(np.array(y))/TICKS_PER_SECOND
    mean_time=np.mean(np.array(y))/TICKS_PER_SECOND
    print("minimal time in trial:" + str(minimal_time) + "s")
    print("median time in trial:" + str(median_time)  + "s")
    print("mean time in trial:" + str(mean_time)  + "s")


    # virtual energy statistics
    print("mean VE= "+str(np.mean(y)))
    print("max VE= " + str(np.max(y)))

    # vfitness statistics
    print("mean F= "+str(np.mean(x_full)))
    print("max F= " + str(np.max(x_full)))
    # plot(x, y, "densityVirtualEnergy.pdf", titles=["HBD"],
    #      axis_names=["Fitness", "Virtual energy + Survived ticks"],
    #      xlim=[-5.0,25.0], ylim=[0, 2000], grid=True)
    createPlot([y],np.array(x),colors,markers,xlabel="Fitness",ylabel="Virtual energy",
               ylim=None,save_filename="scatterVirtualEnergy.pdf",legend_labels=legend_labels,xlim=None,xscale="linear",yscale="linear",
               legendbox=(.10,.10),annotations=[],xticks=[],yticks=[],task_markers=[],scatter=True,
               legend_cols=1,legend_fontsize=26,legend_indexes=[],additional_lines=[],index_x=[],
               xaxis_style="plain",y_err=[],force=False,fill_between=[],
               ax=None, title=None, skip_legend=False)


def scatter_plot():


    x,y,x_full = gather_data_normal()
    create_stats(x,y,x_full)



def scatter_plot_faulty():
    x, y, x_full = gather_data_faulty()
    create_stats(x, y, x_full)





if __name__ == "__main__":
    scatter_plot()