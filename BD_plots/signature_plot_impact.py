
from signature_plot import *



import os
HOME_DIR = os.environ["HOME"]
fault_dir=HOME_DIR+"/argos-sferes/experiments/perturbations"
from perturbation_analysis import *
def get_fault_proportion(fault_type,run,fault_index):
    f = open(fault_dir+"/run"+run+"_p"+fault_index+".txt", 'r')
    lines = f.read().split(',')
    f.close()
    num=0
    if fault_type == "Unique":
        accum=[]
    for line in lines:
        if fault_type=="Proxi":
            if "PROXIMITYSENSORS" in line:
                num+=1
        elif fault_type=="RAB":
            if "RABSENSOR" in line:
                num+=1

        elif fault_type=="Act":
            if "ACTUATOR" in line:
                num+=1
        elif fault_type=="Any":
            if "FAULT_NONE" in line:
                num+=1
        elif fault_type=="Unique":
            if line not in accum:
                num+=1
                accum.append(line)
        else:
            raise Exception("choose either Proxi, RAB, or Act as a fault")

    if fault_type=="Any":  # return the complement of no-fault proportion
        return 1 - num / float(len(lines))
    else:
        return num/float(len(lines))

def get_all_fault_proportions(fault_type):
    """
    :param fault_type the type of fault
    :return proportion of this type in the swarm for each faulty environment
    """
    proportions=[]
    for j in range(len(fitfuns)):
        print(fitfuns[j])
        for fault in range(len(faults)):   # same order of gathering as the impact in significance_data function in perturbation_analysis.py
            for run in runs:
                proportions.append(get_fault_proportion(fault_type,str(run),str(fault)))

    return np.array(proportions)


def plot(xs,ys,name,titles,axis_names,xlim,ylim,grid=False,add_unit_line=False,all=False):
    ## Axis limits for plots of generation 8000

    xmin = xlim[0]  # m1.min()
    xmax = xlim[1]  # m1.max()
    ymin = ylim[0]  # m2.min()
    ymax = ylim[1]  # m2.max()
    # xmin = m1.min()
    # xmax = m1.max()
    # ymin = m2.min()
    # ymax = m2.max()
    X, Y = np.mgrid[xmin:xmax:100j, ymin:ymax:100j]
    positions = np.vstack([X.ravel(), Y.ravel()])

    if all:
            fig = plt.figure(figsize=(10, 10))
            plt.rc('grid', linestyle="dashdot", color='black', alpha=0.50)
            values = np.vstack([xs, ys])
            kernel = gaussian_kde(values)
            Z = np.reshape(kernel(positions).T, X.shape)
            print("max=%.3f  sum=%.3f" % (np.max(Z), np.sum(Z)))

            ax = fig.add_subplot(1, 1, 1)
            ax.grid(grid)
            img = ax.imshow(np.rot90(Z), cmap=plt.cm.gist_earth_r, extent=[xmin, xmax, ymin, ymax], aspect='auto',
                            vmax=8)  # ,vmax=10 'auto'
            ax.tick_params(axis='both', which='major', labelsize=28)
            ax.tick_params(axis='both', which='minor', labelsize=28)
            ax.set_title(titles[0], fontsize=40)

            if add_unit_line:
                x = np.linspace(xmin, xmax, 100)
                y = x
                add_line(x, y, ax)

    else:
        fig = plt.figure(figsize=((len(xs)+1)*10, 10))
        plt.rc('grid', linestyle="dashdot", color='black', alpha=0.50)
        for i in range(len(xs)):
            values = np.vstack([xs[i],ys[i]])
            kernel = gaussian_kde(values)
            Z = np.reshape(kernel(positions).T, X.shape)
            print("max=%.3f  sum=%.3f"%(np.max(Z), np.sum(Z)))

            ax = fig.add_subplot(1,len(xs)+1,i+1)
            ax.grid(grid)
            img = ax.imshow(np.rot90(Z), cmap=plt.cm.gist_earth_r, extent=[xmin, xmax, ymin, ymax], aspect='auto',
                             vmax=8)  # ,vmax=10 'auto'
            ax.tick_params(axis='both', which='major', labelsize=28)
            ax.tick_params(axis='both', which='minor', labelsize=28)
            ax.set_title(titles[i],fontsize=40)

            if add_unit_line:
                x = np.linspace(xmin, xmax, 100)
                y = x
                add_line(x, y, ax)



    num_squares=100.0*100.0
    area_size=(ymax-ymin)*(xmax-xmin)/num_squares
    rounded_max=5.0
    print("chosen % = " +str(100*rounded_max*area_size))

    # vmax=50 or whatever is max across all plots, the max. colorbar tick label is then set to '> vmax/max sum * 100 '

    # plt.plot(m1, m2, 'k.', markersize=2)

    #plt.axis([-1, 0.05, 0, 1])
    #plt.xticks([-2, -1.5, -1, -0.5, 0], ('-2', '-1.5', '-1', '-0.5', '0'))
    #ax.set_yticks([0, .2, .4, .6, .8, 1], ('0', '.2', '.4', '.6', '.8', '1'))


    # add an axes, lower left corner in [0.83, 0.1] measured in figure coordinate with axes width 0.02 and height 0.8

    cb_ax = fig.add_axes([0.80, 0.1, 0.02, 0.7])
    cbar = plt.colorbar(img,cax=cb_ax)
    cbar.ax.set_ylabel('% of solutions',fontsize=36)
    cbar.set_ticks([0.05, rounded_max/2.0, rounded_max])
    cbar.ax.set_yticklabels(['0%', '0.025%', ">0.050%"],fontsize=25)

    fig.text(0.43, 0.010, axis_names[0], ha='center', fontsize=36)
    fig.text(0.090, 0.5, axis_names[1], va='center', rotation='vertical', fontsize=36)
    plt.savefig("results/fault/density/"+name+".pdf")
    plt.close()

if __name__ == "__main__":
    bds=["history","Gomes_sdbc_walls_and_robots_std","cvt_rab_spirit","environment_diversity"]
    impacts=[]
    dists=[]
    resiliences=[]
    self_dists=[]

    correlations=[]
    corr_impact_res_s=[]
    unique_proportions = get_all_fault_proportions("Unique")
    any_proportions = get_all_fault_proportions("Any")

    proxi_proportions = get_all_fault_proportions("Proxi")

    rab_proportions = get_all_fault_proportions("RAB")

    act_proportions = get_all_fault_proportions("Act")

    all_impacts=[]
    all_resiliences=[]
    all_any_proportions=[]
    all_unique_proportions = []
    all_proxi_proportions=[]
    all_rab_proportions=[]
    all_act_proportions=[]

    slopes=[]
    slopes_impact_res=[]
    for i, bd in enumerate(bds):
        resilience, dist, self_dist = get_data(metric="maxvar", bd=bd, fitfuns=["Aggregation", "Dispersion", "DecayCoverage", "DecayBorderCoverage","Flocking"],
                                              history_type= "xy",add_self_dist=True)
        #assert np.allclose(resilience, resilience_data[i])
        #indexes=np.where((best_transfer_data[i] >= -0.4))
        indexes=np.where((best_transfer_data[i] >= -0.5))
        impacts.append(best_transfer_data[i][indexes])
        resiliences.append(resilience[indexes])
        dists.append(dist[indexes])
        self_dists.append(self_dist[indexes])

        r = np.corrcoef(impacts[i],dists[i])[1,0]
        correlations.append(r)
        sx = np.std(impacts[i])
        sy = np.std(dists[i])
        a = r*(sy / sx)
        slopes.append(a)
        r_2 = np.corrcoef(impacts[i], resiliences[i])[1, 0]
        corr_impact_res_s.append(r_2)#
        sy = np.std(resiliences[i])
        a2 = r_2*(sy / sx)
        slopes_impact_res.append(a2)
        all_unique_proportions = np.append(all_unique_proportions, unique_proportions)
        all_any_proportions = np.append(all_any_proportions,any_proportions)
        all_proxi_proportions = np.append(all_proxi_proportions, proxi_proportions)
        all_rab_proportions = np.append(all_rab_proportions, rab_proportions)
        all_act_proportions=np.append(all_act_proportions,act_proportions)
        all_impacts=np.append(all_impacts,impacts[i])
        all_resiliences=np.append(all_resiliences,resiliences[i])
    fit_unique_proportions = []
    fit_any_proportions=[]
    fit_proxi_proportions=[]
    fit_rab_proportions=[]
    fit_act_proportions=[]
    N=len(faults)*len(runs)
    fit_impacts=[]
    fit_resiliences=[]

    for i, f in enumerate(fitfuns):
        print(f)
        id=range(i*N,(i+1)*N)
        fit_impacts.append([])
        fit_resiliences.append([])
        fit_unique_proportions.append([])
        fit_any_proportions.append([])
        fit_proxi_proportions.append([])
        fit_rab_proportions.append([])
        fit_act_proportions.append([])
        for j, bd in enumerate(bds):
            print(bd)
            #assert np.allclose(resilience, resilience_data[i])
            fit_impacts[i]=np.append(fit_impacts[i],best_transfer_data[j][id])
            fit_resiliences[i] = np.append(fit_resiliences[i], resilience_data[j][id])
            fit_unique_proportions[i] = np.append(fit_unique_proportions[i], unique_proportions[id])
            fit_any_proportions[i] = np.append(fit_any_proportions[i], any_proportions[id])
            fit_proxi_proportions[i] = np.append(fit_proxi_proportions[i], proxi_proportions[id])
            fit_rab_proportions[i] = np.append(fit_rab_proportions[i], rab_proportions[id])
            fit_act_proportions[i] = np.append(fit_act_proportions[i], act_proportions[id])
    # plot([unique_proportions]*len(bds), impacts,"impact_unique_signature",titles=["HBD","SDBC","SPIRIT","QED"],
    #      axis_names=["% Unique faults","Impact of fault"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True)
    # plot([any_proportions]*len(bds), impacts,"impact_any_signature",titles=["HBD","SDBC","SPIRIT","QED"],
    #      axis_names=["% Faults","Impact of fault"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True)
    #
    # plot([proxi_proportions]*len(bds), impacts,"impact_proxi_signature",titles=["HBD","SDBC","SPIRIT","QED"],
    #      axis_names=["% Proximity faults","Impact of fault"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True)
    # plot([rab_proportions]*len(bds), impacts,"impact_rab_signature",titles=["HBD","SDBC","SPIRIT","QED"],
    #      axis_names=["% RAB faults","Impact of fault"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True)
    # plot([act_proportions]*len(bds), impacts,"impact_act_signature",titles=["HBD","SDBC","SPIRIT","QED"],
    #      axis_names=["% Actuator faults","Impact of fault"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True)
    #
    # plot([unique_proportions]*len(bds), resiliences,"res_unique_signature",titles=["HBD","SDBC","SPIRIT","QED"],
    #      axis_names=["% Unique faults","Resilience"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True)
    # plot([any_proportions]*len(bds), resiliences,"res_any_signature",titles=["HBD","SDBC","SPIRIT","QED"],
    #      axis_names=["% Faults","Resilience"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True)
    # plot([proxi_proportions]*len(bds), resiliences,"res_proxi_signature",titles=["HBD","SDBC","SPIRIT","QED"],
    #      axis_names=["% Proximity faults","Resilience"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True)
    # plot([rab_proportions]*len(bds), resiliences,"res_rab_signature",titles=["HBD","SDBC","SPIRIT","QED"],
    #      axis_names=["% RAB faults","Resilience"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True)
    # plot([act_proportions]*len(bds), resiliences,"res_act_signature",titles=["HBD","SDBC","SPIRIT","QED"],
    #      axis_names=["% Actuator faults","Resilience"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True)
    # # ######### combine ####
    # #
    # plot(all_unique_proportions, all_impacts,"impact_unique_all",titles=[""],
    #      axis_names=["% Unique faults","Impact of fault"],
    #      xlim=[0.0, 1.0], ylim=[-1, 0.2], grid=True, all=True)
    # plot(all_any_proportions, all_impacts,"impact_any_all",titles=[""],
    #      axis_names=["% Faults","Impact of fault"],
    #      xlim=[0.0, 1.0], ylim=[-1, 0.2], grid=True, all=True)
    # plot(all_proxi_proportions, all_impacts,"impact_proxi_all",titles=[""],
    #      axis_names=["% Proximity faults","Impact of fault"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True,all=True)
    # plot(all_rab_proportions, all_impacts,"impact_rab_all",titles=[""],
    #      axis_names=["% RAB faults","Impact of fault"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True,all=True)
    # plot(all_act_proportions, all_impacts,"impact_act_all",titles=[""],
    #      axis_names=["% Actuator faults","Impact of fault"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True,all=True)
    # #
    # #
    # plot(all_unique_proportions, all_resiliences,"res_unique_all",titles=[""],
    #      axis_names=["% Unique faults","Resilience"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True,all=True)
    # plot(all_any_proportions, all_resiliences,"res_any_all",titles=[""],
    #      axis_names=["% Faults","Resilience"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True,all=True)
    # plot(all_proxi_proportions, all_resiliences,"res_proxi_all",titles=[""],
    #      axis_names=["% Proximity faults","Resilience"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True,all=True)
    # plot(all_rab_proportions, all_resiliences,"res_rab_all",titles=[""],
    #      axis_names=["% RAB faults","Resilience"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True,all=True)
    # plot(all_act_proportions, all_resiliences,"res_act_all",titles=[""],
    #      axis_names=["% Actuator faults","Resilience"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True,all=True)
    # ######### fitness ####
    # plot(fit_unique_proportions, fit_impacts,"impact_unique_fitness",titles=fitfunlabels,
    #      axis_names=["% Unique faults","Impact of fault"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True,all=False)
    # plot(fit_any_proportions, fit_impacts,"impact_any_fitness",titles=fitfunlabels,
    #      axis_names=["% Faults","Impact of fault"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True,all=False)
    # plot(fit_proxi_proportions, fit_impacts,"impact_proxi_fitness",titles=fitfunlabels,
    #      axis_names=["% Proximity faults","Impact of fault"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True,all=False)
    # plot(fit_rab_proportions, fit_impacts,"impact_rab_fitness",titles=fitfunlabels,
    #      axis_names=["% RAB faults","Impact of fault"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True,all=False)
    # plot(fit_act_proportions, fit_impacts,"impact_act_fitness",titles=fitfunlabels,
    #      axis_names=["% Actuator faults","Impact of fault"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True,all=False)
    #
    #
    # plot(fit_unique_proportions, fit_resiliences,"res_unique_fitness",titles=fitfunlabels,
    #      axis_names=["% Faults","Resilience"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True,all=False)
    # plot(fit_any_proportions, fit_resiliences,"res_any_fitness",titles=fitfunlabels,
    #      axis_names=["% Faults","Resilience"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True,all=False)
    # plot(fit_proxi_proportions, fit_resiliences,"res_proxi_fitness",titles=fitfunlabels,
    #      axis_names=["% Proximity faults","Resilience"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True,all=False)
    # plot(fit_rab_proportions, fit_resiliences,"res_rab_fitness",titles=fitfunlabels,
    #      axis_names=["% RAB faults","Resilience"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True,all=False)
    # plot(fit_act_proportions, fit_resiliences,"res_act_fitness",titles=fitfunlabels,
    #      axis_names=["% Actuator faults","Resilience"],
    #      xlim=[0.0,1.0],ylim=[-1,0.2],grid=True,all=False)

    #####################
    # plot(impacts,dists,"impact_distance_signatureNOFLOCK",titles=["HBD","SDBC","SPIRIT","QED"],
    #      axis_names=["Impact of fault","Behavioural diversity"],
    #      xlim=[-0.8,0.2],ylim=[0,1],grid=True)
    #
    # plot(impacts,resiliences,"impact_resilience_signatureNOFLOCK",titles=["HBD","SDBC","SPIRIT","QED"],
    #      axis_names=["Impact of fault","Map resilience"],
    #      xlim=[-0.8,0.2],ylim=[-0.4,0.1],grid=True,add_unit_line=True)
    #
    # plot(self_dists, dists, "projection_signatureNOFLOCK", titles=["HBD", "SDBC", "SPIRIT", "QED"],
    #      axis_names=["distance in own space","distance in projected space"],
    #      xlim=[0, 1], ylim=[0, 1], grid=True,add_unit_line=True)
    #
    # plot(resiliences, dists, "diversity_resilience_signatureNOFLOCK", titles=["HBD", "SDBC", "SPIRIT", "QED"],
    #      axis_names=["Map resilience","Behavioural diversity"],
    #      xlim=[-0.4, 0.1], ylim=[0,1], grid=True, add_unit_line=False)


    # plot(impacts,dists,"impact_distance_signature",titles=["HBD","SDBC","SPIRIT","QED"],
    #      axis_names=["Impact of fault","Behavioural diversity"],
    #      xlim=[-1.0,0.2],ylim=[0,1],grid=True)
    #
    # plot(impacts,resiliences,"impact_resilience_signature",titles=["HBD","SDBC","SPIRIT","QED"],
    #      axis_names=["Impact of fault","Map resilience"],
    #      xlim=[-1.0,0.2],ylim=[-1.0,0.02],grid=True,add_unit_line=True)
    #
    # plot(self_dists, dists, "projection_signature", titles=["HBD", "SDBC", "SPIRIT", "QED"],
    #      axis_names=["distance in own space","distance in projected space"],
    #      xlim=[0, 1], ylim=[0, 1], grid=True,add_unit_line=True)
    #
    # plot(resiliences, dists, "diversity_resilience_signature", titles=["HBD", "SDBC", "SPIRIT", "QED"],
    #      axis_names=["Map resilience","Behavioural diversity"],
    #      xlim=[-1.0, 0.2], ylim=[0,1], grid=True, add_unit_line=False)


    plot(impacts,dists,"impact_distance_signature",titles=["HBD","SDBC","SPIRIT","QED"],
         axis_names=["Impact of fault","Behavioural diversity"],
         xlim=[-0.5,0.0],ylim=[0,1],grid=True)

    plot(impacts,resiliences,"impact_resilience_signature",titles=["HBD","SDBC","SPIRIT","QED"],
         axis_names=["Impact of fault","Map resilience"],
         xlim=[-0.5,0.0],ylim=[-1.0,0.02],grid=True,add_unit_line=True)

    plot(self_dists, dists, "projection_signature", titles=["HBD", "SDBC", "SPIRIT", "QED"],
         axis_names=["distance in own space","distance in projected space"],
         xlim=[0, 1], ylim=[0, 1], grid=True,add_unit_line=True)

    plot(resiliences, dists, "diversity_resilience_signature", titles=["HBD", "SDBC", "SPIRIT", "QED"],
         axis_names=["Map resilience","Behavioural diversity"],
         xlim=[-0.5, 0.0], ylim=[0,1], grid=True, add_unit_line=False)