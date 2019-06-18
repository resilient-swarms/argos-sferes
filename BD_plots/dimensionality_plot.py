
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import seaborn as sns
import time
import sys,os
HOME_DIR = os.environ["HOME"]
RESULTSFOLDER="results"


from pandas.plotting import scatter_matrix
from process_archive_data import *

from sklearn.decomposition import PCA
from sklearn.manifold import TSNE

from mpl_toolkits.mplot3d import Axes3D

from matplotlib.colors import ListedColormap

# construct cmap
flatui = ["#9b59b6", "#3498db", "#95a5a6", "#e74c3c", "#34495e", "#2ecc71"]
my_cmap = ListedColormap(sns.color_palette(flatui).as_hex())


labels=["sec1","sec2","sec3","sec4","sec5","sec6"]
NCOLORS=5

def create_scatter_matrix(data_matrix, dimnames,savefile):
    """

    :param data_matrix: a (N,dims) matrix, where N is the total number of observations, and dims the number of dimensions
    :param dimnames:
    :return:
    """
    df = pd.DataFrame(data_matrix, columns=dimnames)
    fig = scatter_matrix(df, alpha=0.2, figsize=(5, 5))
    for i in range(fig.shape[0]):
        for j in range(fig.shape[1]):
            fig[i][j].set_xlim([0.0, 1.0])
            fig[i][j].set_ylim([0.0,1.0])
    fig = fig[0][0].get_figure()
    fig.tight_layout()
    fig.savefig(savefile)  #,dpi=50


def create_BD_scatter(BD_directory, runs, archive_file_path,dimnames,savefile):
    data_matrix = np.array(get_combined_archive(BD_directory, runs, archive_file_path,by_bin=False))
    create_scatter_matrix(data_matrix,dimnames,savefile)


def colorFromBivariateData(Z1,Z2,cmap1 = plt.cm.YlOrRd, cmap2 = plt.cm.PuBuGn):
    # Rescale values to fit into colormap range (0->255)
    Z1_plot = np.array(255*(Z1-Z1.min())/(Z1.max()-Z1.min()), dtype=np.int)
    Z2_plot = np.array(255*(Z2-Z2.min())/(Z2.max()-Z2.min()), dtype=np.int)

    Z1_color = cmap1(Z1_plot)
    Z2_color = cmap2(Z2_plot)

    # Color for each point
    Z_color = np.sum([Z1_color, Z2_color], axis=0)/2.0

    return Z_color





def direct_scatter(data_path,runs, archive_file_path,bd_labels,savefile):
    bd = np.array(get_combined_archive(data_path, runs, archive_file_path, by_bin=False, include_val=False))

    #colors = np.linspace(0, 1, len(bd[:, 0]))
    colors = [0 for i in bd[:,0]]
    plt.figure(figsize=(16, 10))

    fig = plt.figure()

    ax = fig.add_subplot(111)

    ax.scatter(bd[:, 0],bd[:, 1], c=colors, cmap=my_cmap)

    plt.xlabel(bd_labels[0])
    plt.ylabel(bd_labels[1])
    plt.savefig(savefile)


def reduce_PCA(components,data_path,runs, archive_file_path,bd_labels,plot,savefile):
    """

    :param bd: the different behaviours and its descriptors
    :param bd_labels: the labels of each behavioural dimension
    :return:
    """
    bd = np.array(get_combined_archive(data_path, runs, archive_file_path,by_bin=False,include_val=False))
    df = pd.DataFrame(bd, columns=bd_labels)
    pca = PCA(n_components=components)
    pca_result = pca.fit_transform(df[bd_labels].values)

    if not plot:
        return df


    #
    #colors = np.linspace(0, 1, len(pca_result[:, 0]))

    colors = [sorting_function(b) for b in bd]
    plt.figure(figsize=(16, 10))

    fig = plt.figure()
    if components==3:
        ax = fig.add_subplot(111, projection='3d')

        scat = ax.scatter(pca_result[:, 0], pca_result[:, 1], pca_result[:, 2],c=colors,
                   cmap=my_cmap)
    else:
        ax = fig.add_subplot(111)

        scat = ax.scatter(pca_result[:, 0], pca_result[:, 1], c=colors,
                   cmap=my_cmap)
    color_bar(6, scat)
    ax.legend()
    plt.savefig(savefile)


def color_bar(N,scat):
    # create the colorbar for a scatter plot
    bounds = np.linspace(0, 1.0, N + 1)
    cb = plt.colorbar(scat, spacing='proportional', ticks=bounds)
    cb.set_label('Custom cbar')

def reduce_tSNE(components,data_path,runs, archive_file_path,bd_labels,plot,savefile):
    bd = np.array(get_combined_archive(data_path, runs, archive_file_path, by_bin=False, include_val=False))
    if len(bd_labels)>400:
        df = reduce_PCA(components=400,data_path=data_path,runs=runs, archive_file_path=archive_file_path,
                        bd_labels=bd_labels,plot=False,savefile="")
    else:
        df = pd.DataFrame(bd, columns=bd_labels)

    colors = [sorting_function(b) for b in bd]
    time_start = time.time()
    tsne = TSNE(n_components=components, verbose=1, perplexity=60, n_iter=500)
    tsne_results = tsne.fit_transform(df)

    # df['tsne-2d-one'] = tsne_results[:, 0]
    # df['tsne-2d-two'] = tsne_results[:, 1]
    plt.figure(figsize=(16, 10))

    fig = plt.figure()
    if components==3:
        ax = fig.add_subplot(111, projection='3d')

        scat = ax.scatter(tsne_results[:, 0], tsne_results[:, 1],  tsne_results[:, 2],c=colors,
                   cmap=my_cmap)
    else:
        ax = fig.add_subplot(111)

        scat =ax.scatter(tsne_results[:, 0], tsne_results[:, 1], c=colors, cmap=my_cmap)

    color_bar(6,scat)
    plt.legend()
    # sns.scatterplot(
    #     x=tsne_results[:, 0], y=tsne_results[:, 1],
    #     palette=sns.cubehelix_palette(as_cmap=True),
    #     data=df,
    #     legend="full",
    #     alpha=0.3
    # )
    plt.savefig(RESULTSFOLDER+"/"+savefile)

    print('t-SNE done! Time elapsed: {} seconds'.format(time.time() - time_start))






if __name__ == "__main__":

    # create_BD_scatter(HOME_DIR+"/argos-sferes/experiments/data/MeanSpeed/history", 5,
    #                   "archive_900.dat",["uniformity","deviation","coverage","F"],savefile="jointBDillustration.pdf")
    # create_BD_scatter(HOME_DIR+"/argos-sferes/experiments/data/MeanSpeed/sdbc_walls_and_robots", 5,
    #                   "archive_900.dat", [r"$x$", r"$y$", r"$\theta$", r"$V_{left}$",r"$V_{right}$","dispersion","F"],
    #                   savefile="jointBDillustration2.png")
    #data_matrix = np.array(get_combined_archive((, 5,
    #                   "archive_900.dat",["uniformity","deviation","coverage","F"],savefile="jointBDillustration.pdf"))

    methods=["history","cvt_mutualinfo","cvt_mutualinfoact","cvt_spirit"]
    dimensionality=[2,21,14,400]
    for i in range(len(methods)):
        method=methods[i]
        dim=dimensionality[i]
        if dim==2:
            direct_scatter(data_path=HOME_DIR + "/Data/datanew/Coveragerange50/"+method, runs=5,
                   archive_file_path="archive_1200.dat",
                   bd_labels=["uniformity","deviation"], savefile=method+"scatter.pdf")
            continue
        reduce_PCA(components=2, data_path=HOME_DIR + "/Data/datanew/Coveragerange50/"+method, runs=5,
                   archive_file_path="archive_1200.dat",
                   bd_labels=["BD" + str(i) for i in range(dim)], plot=True, savefile="PCA_"+method+"2D.pdf")

        reduce_tSNE(components=2,data_path=HOME_DIR + "/Data/datanew/Coveragerange50/"+method, runs=5,
                    archive_file_path="archive_1200.dat",
                    bd_labels=["BD" + str(i) for i in range(dim)], plot=True, savefile="tSNE_"+method+"2D.pdf")

        reduce_PCA(components=3,data_path=HOME_DIR+"/Data/datanew/Coveragerange50/"+method, runs=5, archive_file_path="archive_1200.dat",
                   bd_labels=["BD"+str(i) for i in range(dim)], plot=True, savefile="PCA_"+method+"3D.pdf")

        reduce_tSNE(components=3,data_path=HOME_DIR+"/Data/datanew/Coveragerange50/"+method, runs=5, archive_file_path="archive_1200.dat",
                   bd_labels=["BD"+str(i) for i in range(dim)], plot=True, savefile="tSNE_"+method+"3D.pdf")