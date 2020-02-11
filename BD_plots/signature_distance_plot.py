# plot the behavioural distance in SPIRIT space as a function of behavioural distance in environment space

from signature_plot import *

def get_descriptor_data():
    """]
    for fitfun in fitfuns:
        for run in runs:
            #get the best individual for the normal environment

            #get the best individual for the faulty environments

            #get the distance
    :return:
    """

def plot(xs, ys, name, titles, axis_names, xlim, ylim, grid=False):
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
    fig = plt.figure(figsize=(50, 10))
    plt.rc('grid', linestyle="dashdot", color='black', alpha=0.50)
    for i in range(len(xs)):
        values = np.vstack([xs[i], ys[i]])
        kernel = gaussian_kde(values)
        Z = np.reshape(kernel(positions).T, X.shape)
        print("max=%.3f  sum=%.3f" % (np.max(Z), np.sum(Z)))

        ax = fig.add_subplot(1, len(xs) + 1, i + 1)
        ax.grid(grid)
        img = ax.imshow(np.rot90(Z), cmap=plt.cm.gist_earth_r, extent=[xmin, xmax, ymin, ymax], aspect='auto',
                        vmax=8)  # ,vmax=10 'auto'
        ax.tick_params(axis='both', which='major', labelsize=28)
        ax.tick_params(axis='both', which='minor', labelsize=28)
        ax.set_title(titles[i], fontsize=40)

    num_squares = 100.0 * 100.0
    area_size = (ymax - ymin) * (xmax - xmin) / num_squares
    rounded_max = 6.0
    print("chosen % = " + str(100 * rounded_max * area_size))

    # vmax=50 or whatever is max across all plots, the max. colorbar tick label is then set to '> vmax/max sum * 100 '

    # plt.plot(m1, m2, 'k.', markersize=2)

    # plt.axis([-1, 0.05, 0, 1])
    # plt.xticks([-2, -1.5, -1, -0.5, 0], ('-2', '-1.5', '-1', '-0.5', '0'))
    # ax.set_yticks([0, .2, .4, .6, .8, 1], ('0', '.2', '.4', '.6', '.8', '1'))

    # add an axes, lower left corner in [0.83, 0.1] measured in figure coordinate with axes width 0.02 and height 0.8

    cb_ax = fig.add_axes([0.75, 0.1, 0.02, 0.7])
    cbar = plt.colorbar(img, cax=cb_ax)
    cbar.ax.set_ylabel('% of solutions', fontsize=36)
    cbar.set_ticks([0.05, rounded_max / 2.0, rounded_max])
    cbar.ax.set_yticklabels(['0%', '0.015%', ">0.030%"], fontsize=25)

    fig.text(0.43, 0.015, axis_names[0], ha='center', fontsize=36)
    fig.text(0.095, 0.5, axis_names[1], va='center', rotation='vertical', fontsize=36)
    plt.savefig("results/fault/density/" + name + ".pdf")
    plt.close()

if __name__ == "__main__":
    bds=["history","Gomes_sdbc_walls_and_robots_std","cvt_rab_spirit","environment_diversity"]
    xs=[]
    ys=[]
    for bd in bds:
        _unused,y=get_data("maxvar",bd,["Aggregation","Dispersion","DecayCoverage","DecayBorderCoverage","Flocking"],"xy")
        x=get_descriptor_data(bd)
        ys.append(y)
    plot(xs,ys,"signature_distance",titles=["HBD","SDBC","SPIRIT","QED"],
         axis_names=["distance in own space","distance in projected space"],
         xlim=[0,1],ylim=[0,1],grid=True)