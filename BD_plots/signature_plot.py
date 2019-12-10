
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
from perturbation_analysis import *

loadfilename="data/combined/summary_statistics.pkl"
best_performance_data, performance_data, best_transfer_data, transfer_data, resilience_data = pickle.load(
            open(loadfilename, "rb"))
def bin_distances(fitfun,bd,history_type):
    """
    print normal_bd_self
    then bin distances of other bds and look at the different clusters
    :return:
    """
    normal_bd_file, normal_bd_self_file, faulty_bd_file, faulty_bd_self_file = bd_filenames(fitfun, bd, history_type)
    normal_bd_self = pickle.load(open(normal_bd_self_file,"rb"))

    faulty_bd_self = pickle.load(open(faulty_bd_self_file,"rb"))

    print()
def get_data(metric,bd,fitfuns,history_type,add_self_dist=False):
        print(bd)
        y = []
        x = []
        self_dists=[]
        for fitfun in fitfuns:
            print(fitfun)
            dp_file, _, _, _, _, _,_ = filenames(fitfun, bd, history_type)
            euclid_file, maxvar_file, category_file, category_h_file, selfdistfile = unperturbed_filenames(fitfun, bd, history_type)
            performances, nofaultperfs = pickle.load(open(dp_file, "rb"))
            dps = np.array(performances) - np.array(nofaultperfs)


            categories = pickle.load(open(category_file, "rb"))
            categories_h = pickle.load(open(category_h_file, "rb"))

            if metric=="euclid":
                euclids = pickle.load(open(euclid_file, "rb"))
                y = np.append(y, euclids)
            elif metric=="maxvar":
                mv = pickle.load(open(maxvar_file, "rb"))
                y = np.append(y, mv)
            elif metric=="category_h":
                ch = pickle.load(open(category_h_file, "rb"))
                y = np.append(y, ch)
            elif metric=="category":
                c = pickle.load(open(category_file, "rb"))
                y = np.append(y, c)
            else:
                raise Exception("not implemented")
            x = np.append(x, np.array(dps) / np.array(nofaultperfs))

            if add_self_dist:
                self_d = pickle.load(open(selfdistfile,"rb"))
                self_dists = np.append(self_dists, self_d)
            bin_distances(fitfun,bd,history_type)
        return x,y,self_dists
def plot(xs,ys,name,titles,axis_names,xlim,ylim,grid=False):
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
    fig = plt.figure(figsize=(50,10))
    plt.rc('grid', linestyle="dashdot", color='black',alpha=0.50)
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




    num_squares=100.0*100.0
    area_size=(ymax-ymin)*(xmax-xmin)/num_squares
    rounded_max=6.0
    print("chosen % = " +str(100*rounded_max*area_size))

    # vmax=50 or whatever is max across all plots, the max. colorbar tick label is then set to '> vmax/max sum * 100 '

    # plt.plot(m1, m2, 'k.', markersize=2)

    #plt.axis([-1, 0.05, 0, 1])
    #plt.xticks([-2, -1.5, -1, -0.5, 0], ('-2', '-1.5', '-1', '-0.5', '0'))
    #ax.set_yticks([0, .2, .4, .6, .8, 1], ('0', '.2', '.4', '.6', '.8', '1'))


    # add an axes, lower left corner in [0.83, 0.1] measured in figure coordinate with axes width 0.02 and height 0.8

    cb_ax = fig.add_axes([0.75, 0.1, 0.02, 0.7])
    cbar = plt.colorbar(img,cax=cb_ax)
    cbar.ax.set_ylabel('% of solutions',fontsize=36)
    cbar.set_ticks([0.05, rounded_max/2.0, rounded_max])
    cbar.ax.set_yticklabels(['0%', '0.015%', ">0.030%"],fontsize=25)

    fig.text(0.43, 0.015, axis_names[0], ha='center', fontsize=36)
    fig.text(0.095, 0.5, axis_names[1], va='center', rotation='vertical', fontsize=36)
    plt.savefig("results/fault/density/"+name+".pdf")
    plt.close()

if __name__ == "__main__":
    bds=["history","Gomes_sdbc_walls_and_robots_std","cvt_rab_spirit","environment_diversity"]
    xs=[]
    ys=[]
    for i, bd in enumerate(bds):
        x,y,_unused=get_data("maxvar",bd,["Aggregation","Dispersion","DecayCoverage","DecayBorderCoverage","Flocking"],"xy")
        assert np.allclose(x,resilience_data[i])
        xs.append(x)
        ys.append(y)
    plot(xs,ys,"maxvar_blackgridnew",titles=["HBD","SDBC","SPIRIT","QED"],
         axis_names=["Map resilience","Behavioural diversity"],
         xlim=[-0.4,0.1],ylim=[0,1],grid=True)