
import numpy as np 
import matplotlib.pyplot as PLT
from scipy.stats import *

# ================    ===============================
# character           description
# ================    ===============================
#    -                solid line style
#    --               dashed line style
#    -.               dash-dot line style
#    :                dotted line style
#    .                point marker
#    ,                pixel marker
#    o                circle marker
#    v                triangle_down marker
#    ^                triangle_up marker
#    <                triangle_left marker
#    >                triangle_right marker
#    1                tri_down marker
#    2                tri_up marker
#    3                tri_left marker
#    4                tri_right marker
#    s                square marker
#    p                pentagon marker
#    *                star marker
#    h                hexagon1 marker
#    H                hexagon2 marker
#    +                plus marker
#    x                x marker
#    D                diamond marker
#    d                thin_diamond marker
#    |                vline marker
#    _                hline marker
# ================    ===============================

def newline_latex(f,add_hline=False):

    if add_hline:
        f.write(r"\\ \hline")
    else:
        f.write(r"\\ ")
    f.write("\n")
def table_entry_rowcondition(f, rowlabel):
    f.write(rowlabel)

def table_entry_label(f, label):
    f.write(r" & %s " % (label))

def table_entry_meansd(f, stat , type="float3"):
    if type=="float3":
        f.write(r" & $%.3f \pm %.2f$ " % (np.mean(stat), np.std(stat)))
    elif type=="float2":
        f.write(r" & $%.2f \pm %.2f$ " % (np.mean(stat), np.std(stat)))
    else:
        f.write(r" & $%d \pm %d$ " % (np.mean(stat), np.std(stat)))
def IQR(x):
    return np.quantile(x, 0.75) - np.quantile(x, 0.25)
def table_entry_median(f, stat , type="float3"):
    if type=="float3":
        f.write(r" & $%.3f \pm %.2f$ " % (np.median(stat), IQR(stat)))#iqr(stat)))
    elif type=="float2":
        f.write(r" & $%.2f \pm %.2f$ " % (np.median(stat), IQR(stat))) #iqr(stat)))
    else:
        f.write(r" & $%d \pm %d$ " % (np.median(stat), IQR(stat)))#iqr(stat)))
def make_table(f,stats,rowlabels,columnlabels, conditionalcolumnlabels=[],median=False,transpose=False):

    n=len(conditionalcolumnlabels)



    # top-level labels
    if columnlabels:
        for label in columnlabels:
            f.write(r"   & \multicolumn{"+str(n)+"}{c}{%s}  " % (label))
    newline_latex(f,add_hline=True)

    # second-level labels
    if columnlabels:
        for i in range(len(columnlabels)):
            for j in range(len(conditionalcolumnlabels)):
                table_entry_label(f,conditionalcolumnlabels[j][0])
    else:
        for j in range(len(conditionalcolumnlabels)):
                table_entry_label(f, conditionalcolumnlabels[j][0])

    newline_latex(f)


    # second-level labels

    if columnlabels:
        for i in range(len(rowlabels)):
            print(rowlabels[i])
            table_entry_rowcondition(f,rowlabels[i])
            for j in range(len(columnlabels)):
                print(columnlabels[j])
                for k in range(len(conditionalcolumnlabels)): # needs some fix here
                    print(conditionalcolumnlabels[k])


                    if not transpose:
                        stat = stats[k][j][i]
                    else:
                        stat = stats[k][i][j]
                    if median:
                        table_entry_median(f, stat, conditionalcolumnlabels[k][1])
                    else:
                        table_entry_meansd(f,stat,conditionalcolumnlabels[k][1])
            newline_latex(f)
    else:
        for i in range(len(rowlabels)):
            table_entry_rowcondition(f,rowlabels[i])
            for j in range(len(conditionalcolumnlabels)):
                if not transpose:
                    stat = stats[j][i]
                else:
                    stat = stats[i][j]
                if median:
                    table_entry_median(f, stat, conditionalcolumnlabels[j][1])
                else:
                    table_entry_meansd(f,stat,conditionalcolumnlabels[j][1])
            newline_latex(f)



# def table_row(f,stats,labels,conditionallabels):
#     """
#
#     :param f:
#     :param stats:
#     :param labels:
#     :param conditionallabels:
#     :return:
#     """
#     # second-level labels
#     for _ in labels:
#         for label in conditionallabels:
#             f.write(r"   & %s  " % (label))
#
#     newline_latex(f)



def createBarplot(stats,x_values,colors,markers,xlabel,ylabel,ylim,save_filename,legend_labels,yscale="linear",
               legendbox=(.10,.10),legend_indexes=[]):
    print("prepare plotting barplot "+ylabel)
    skip=input("skip y/n")
    if skip=="y":
        return
    print("start plotting "+str(ylabel))
    print("")
    figx=10
    figy=10
    if legend_indexes:
        legend_labels = [legend_labels[i] for i in legend_indexes]
        markers = [markers[i] for i in legend_indexes]
        colors = [colors[i] for i in legend_indexes]
    import matplotlib.pyplot as plt
    while True:
        # ylim: [0,y_max]

        fig = plt.figure(figsize=(figx,figy))
        ax = fig.add_subplot(1, 1, 1)
        ax.tick_params(axis='both', which='major', labelsize=28)
        ax.tick_params(axis='both', which='minor', labelsize=28)
        axes = fig.gca()
        axes.set_xlabel(xlabel, fontsize=36)
        axes.set_ylabel(ylabel, fontsize=36)
        ax.ticklabel_format(style='sci', axis='x', scilimits=(0, 0))
        ax.xaxis.offsetText.set_fontsize(28)

        axes.set_ylim(ylim)
        count=0
        lines=[]
        cols=[]
        step=x_values[1]-x_values[0]
        width=step/float(len(legend_labels))
        ax.set_xticks(x_values)
        ax.set_xticklabels(x_values)
        x_values=np.array(x_values)

        for index in range(len(legend_labels)):
                cols.append(colors[index])
                ax.bar(x_values+count*width, np.array(stats[index][0:20]),
                       width=width ,align='center', alpha=0.7, label=legend_labels[index], color=colors[index])
                count+=1
                #lines.append(stats[index])
                #lines.append(line)
        plt.yscale(yscale)
        plt.grid(True)

        fig.legend(lines, labels=legend_labels, loc=3, ncol=1, bbox_to_anchor=legendbox, prop={'size': 25},)
        fig.tight_layout()
        fig.show()
        accept = input("accept y/n")
        if accept == "y":
            fig.savefig(save_filename)
            return
        ylim[1] = float(input("maxy"))
        ylim[0] = float(input("miny"))

    # ylim[1] = float(input("maxy"))
    # ylim[0] = float(input("miny"))
    # yscale = str(input("scale: linear/log/symlog/logit"))
    # figx = int(input("figx"))
    # figy = int(input("figy"))
    # legbox_x = float(input("legbox_x"))
    # legbox_y = float(input("legbox_y"))
    # legendbox = (legbox_x, legbox_y)

def xticks_development(ax):
    ax.tick_params(axis='both', which='major', labelsize=28)
    ax.tick_params(axis='both', which='minor', labelsize=28)
    ax.ticklabel_format(style='sci', axis='x', scilimits=(0, 0))
    ax.xaxis.offsetText.set_fontsize(28)
def xticks_additional(ax,xticks):
    ax.set_xticks(xticks)
def add_line(x,y,plt):
    l, = plt.plot(x,y, color='k', linestyle='-', linewidth=2)
    l.set_dashes([2, 2, 10, 2])
    return l

def get_plot(ax,index, xx, stats, colors, markers, markers_on,y_err=[],fill_between=[],legend_label=None):
    if y_err:
        line = ax.errorbar(xx, stats[index],yerr=y_err[index], color=colors[index], marker=markers[index], markevery=markers_on, ms=8,
                    linewidth=4,label=legend_label)
    else:
        line, = ax.plot(xx, stats[index], color=colors[index], marker=markers[index], markevery=markers_on, ms=16,
                            linewidth=4,label=legend_label)


        if fill_between:
            y1,y2=fill_between
            y_temp=y1[index]
            y_temp2=y2[index]
            ax.fill_between(xx, y_temp, y_temp2, where=y_temp2 >= y_temp, facecolor=colors[index], interpolate=True,alpha=0.5)
    return line

def finish_fig(fig,save_filename, colorbar=None):
    if colorbar is not None:
        colorbar,labels = colorbar
        off_x = 0.92
        width = 0.03
        off_y = 0.08
        height = 0.8
        cbaxes = fig.add_axes([off_x,off_y, width, height])  # left, bottom, width, height
        cbar = fig.colorbar(colorbar,cax=cbaxes)
        cbar.ax.tick_params(labelsize=20)

        # for j, lab in enumerate(labels):
        #     cbar.ax.text(1.0, off_y + j*height/float(len(labels)), lab, ha='center', va='center')
        #cbar.set_yticklabels([mn, md, mx])  # add the labels
        cbar.ax.set_ylabel('density', rotation=270,fontsize=29,labelpad=+28)
    else:
        fig.tight_layout()
    fig.savefig(save_filename)



def createPlot(stats,x_values,colors,markers,xlabel,ylabel,ylim,save_filename,legend_labels,xlim=None,xscale="linear",yscale="linear",
               legendbox=(.10,.10),annotations=[],xticks=[],yticks=[],task_markers=[],scatter=False,
               legend_cols=1,legend_fontsize=26,legend_indexes=[],additional_lines=[],index_x=[],
               xaxis_style="plain",y_err=[],force=False,fill_between=[],
               ax=None, title=None, skip_legend=False
               ):
    print("prepare plotting "+ylabel)
    if not force:
        skip=input("skip y/n")
        if skip=="y":
            return
    print("start plotting "+str(ylabel))
    print("")
    figx=15
    figy=10

    # legend_labels=[legend_labels[i] for i in legend_indexes]
    # markers = [markers[i] for i in legend_indexes]
    # colors = [colors[i] for i in legend_indexes]
    while True:
        # ylim: [0,y_max]

        if ax is None:
            fig = PLT.figure(figsize=(figx,figy))
            ax = fig.add_subplot(1,1,1)# # use default : assumes time development plot
        else:
            fig = None


        ax.xaxis.major.formatter._useMathText = True

        ax.ticklabel_format(style=xaxis_style, axis='x')


        lines=[]
        if isinstance(x_values, list):
            is_list=True
        else:
            is_list=False
        for index in range(len(stats)):
            if is_list:
                x = x_values[index]
            else:
                x=x_values
            #num_steps = len(x) if len(x) < 20 else 20
            #interval_width = int(len(x) // num_steps)
            if scatter:
                line = ax.scatter(x, stats[index], color=colors[index], marker=markers[index],s=1500)
            else:
                if index_x:
                    xx=index_x[index]
                else:
                    xx= x if len(stats[index])==len(x) else x[0:len(stats[index])]
                markers_on = range(0, len(xx), 1)
                line=get_plot(ax,index,xx,stats,colors,markers,markers_on,y_err, fill_between,legend_labels[index])

            lines.append(line)
        #axes = PLT.gca()
        ax.set_xlabel(xlabel, fontsize=46)

        ax.set_ylabel(ylabel, fontsize=46)

        if ylim is not None:
            ax.set_ylim(ylim)
        if xlim is not None:
            ax.set_xlim(xlim)

        if title:
            ax.set_title(title, fontsize=50)


        ax.tick_params(axis='both', which='major', labelsize=40)
        ax.tick_params(axis='both', which='minor', labelsize=40)
        ax.xaxis.offsetText.set_fontsize(40)
        ax.yaxis.offsetText.set_fontsize(40)


        for (xc,F) in task_markers:
            ax.axvline(x=xc)
        for x,y in additional_lines:
            add_line(x,y,ax)
        for annot in annotations:
            if "xytext" in annot:
                if isinstance(annot["xy"],list): # annotation refers to multiple points
                    count=0
                    for xy in annot["xy"]:
                        text = "" if count > 0 else annot["text"]
                        ax.annotate("", xy=xy, xytext=annot["xytext"],textcoords="data",xycoords="data",
                                arrowprops=dict(facecolor='black', shrink=0.05), verticalalignment="bottom",horizontalalignment=annot["align"], fontsize=annot["fontsize"] )
                        count+=1
                else:
                    ax.annotate(annot["text"], xy=annot["xy"], xytext=annot["xytext"],textcoords="data",xycoords="data",
                        arrowprops=dict(facecolor='black', shrink=0.05),verticalalignment="bottom",horizontalalignment=annot["align"],fontsize=annot["fontsize"])
            else:
                ax.annotate(annot["text"], xy=annot["xy"],textcoords="data",xycoords="data",verticalalignment="bottom",horizontalalignment=annot["align"],fontsize=annot["fontsize"])
        # loc=3,ncol=1,bbox_to_anchor=(0.10, 0.70),
        ax.set_yscale(yscale)
        if isinstance(xscale,dict):
            PLT.xscale(**xscale)
        else:
            ax.set_xscale(xscale)

        if xticks:
            if isinstance(xticks, dict):
                PLT.xticks(xticks["ticks"], xticks["labels"])
            else:
                ax.set_xticks(xticks)
        if yticks:
            ax.tick_params(
                axis='y',  # changes apply to the x-axis
                which='both',  # both major and minor ticks are affected
                bottom=False,  # ticks along the bottom edge are off
                top=False,  # ticks along the top edge are off
                labelbottom=False)  # labels along the bottom edge are off
            ax.set_yticks(yticks)

        ax.grid(True)
        if not skip_legend:
            leg = ax.legend(lines,labels=legend_labels, loc="best",ncol=legend_cols,
                       prop={'size':legend_fontsize},
                       fancybox=True)
            leg.set_alpha(0.20)

        if fig is None: return # nothing to save, just a subplot
        fig.tight_layout()
        fig.show()
        if force:
            accept="y"
        else:
            accept = input("accept y/n")
        if accept == "y":
            print("saving ",save_filename)
            fig.savefig(save_filename)
            return
        redo = str(input("redo settings ? y/n"))
        if redo == "y":
            ylim[1] = float(input("maxy"))
            ylim[0] = float(input("miny"))
            yscale = str(input("scale: linear/log/symlog/logit"))
            figx = int(input("figx"))
            figy = int(input("figy"))
            legbox_x = float(input("legbox_x"))
            legbox_y = float(input("legbox_y"))
        create_annot = str(input("create/adjust annotation ? y/n"))
        if create_annot == "y":
            added_annot = {}
            added_annot["text"]=str(input("text"))
            num_points=int(input("number of points to annotate"))
            if num_points == 1:
                added_annot["xy"] = tuple(map(float,input("xy").split(',')))
            else:
                added_annot["xy"] = []
                for point in num_points:
                    added_annot["xy"].append(tuple(map(float, input("xy%d"%(point)).split(','))))


            with_arrow = str(input("with_arrow ? y/n"))
            if with_arrow=="y":
                added_annot["xytext"] =tuple(map(float,input("xytext").split(',')))
            annot["align"] = str(input("horizontal alignment"))
            #added_annot["h_align"] = str(input("horizontalalignment"))
            index = int(input("annotation_index"))
            if index + 1 <= len(annotations):
                annotations[index]=added_annot
            else:
                annotations.append(added_annot)


        legendbox=(legbox_x,legbox_y)


def createTwinPlot(stats1, stats2, x_values, colors, markers, xlabel, ylabel, ylim, save_filename, legend_labels, yscale="linear",annotations=[], xticks=[], yticks=[],task_markers=[], scatter=False, legend_cols=1,
               legend_fontsize=26, legend_indexes1=[],legend_indexes2=[],force=False):
    print("prepare plotting " + str(ylabel))
    if not force:
        skip = input("skip y/n")
        if skip == "y":
            return
    print("start plotting " + str(ylabel))
    print("")
    figx = 15
    figy = 10

    legend_labels1 = [legend_labels[i] for i in legend_indexes1]
    markers1 = [markers[i] for i in legend_indexes1]
    colors1 = [colors[i] for i in legend_indexes1]

    legend_labels2 = [legend_labels[i] for i in legend_indexes2]
    markers2 = [markers[i] for i in legend_indexes2]
    colors2 = [colors[i] for i in legend_indexes2]
    while True:
        # ylim: [0,y_max]
        fig = PLT.figure(figsize=(figx, figy))
        PLT.rc('axes', axisbelow=True)
        x = x_values
        step = x_values[1] - x_values[0]

        num_steps = len(x) if len(x) < 20 else 20
        interval_width = len(x) / num_steps

        ax = fig.add_subplot(1, 1, 1)
        ax.set_axisbelow(True)
        ax.set_xlabel(xlabel, fontsize=36)
        # use default : assumes time development plot
        ax.tick_params(axis='both', which='major', labelsize=28)
        ax.tick_params(axis='both', which='minor', labelsize=28)
        ax.ticklabel_format(style='plain', axis='x', scilimits=(0, 0))
        ax.xaxis.offsetText.set_fontsize(28)
        if xticks:
            ax.set_xticks(xticks)

        ax2=ax.twinx()

        axes = fig.gca()


        ax.set_ylabel(ylabel[0], fontsize=36)
        if ylim[0]:
            ax.set_ylim(ylim[0])

        ax2.set_ylabel(ylabel[1], fontsize=36)
        if ylim[1]:
            ax2.set_ylim(ylim[1])

        ax.grid(True)

        # ax.yaxis.grid(False)
        # ax.set_axisbelow(True)
        # ax2.set_axisbelow(True)

        ax2.tick_params(axis='both', which='major', labelsize=28)
        ax2.tick_params(axis='both', which='minor', labelsize=28)


        if yticks:
            ax.set_yticks(yticks[0])
            ax2.set_yticks(yticks[1])
        lines = []

        for index in range(len(stats1)):
            if scatter:
                line = ax.scatter(x, stats1[index], color=colors[index], marker=markers[index], s=1200)
            else:
                xx = x if len(stats1[index]) == len(x) else x[0:len(stats1[index])]

                markers_on = range(0, len(xx), interval_width)
                line, = ax.plot(xx, stats1[index], color=colors1[index], marker=markers1[index], markevery=markers_on,
                                ms=16, linewidth=4)
            lines.append(line)
        for (xc, F) in task_markers:
            axes.axvline(x=xc)

        for annot in annotations:
            if "xytext" in annot:
                if isinstance(annot["xy"], list):  # annotation refers to multiple points
                    count = 0
                    for xy in annot["xy"]:
                        text = "" if count > 0 else annot["text"]
                        ax.annotate("", xy=xy, xytext=annot["xytext"], textcoords="data", xycoords="data",
                                    arrowprops=dict(facecolor='black', shrink=0.05), verticalalignment="bottom",
                                    horizontalalignment=annot["align"], fontsize=annot["fontsize"])
                        count += 1
                else:
                    ax.annotate(annot["text"], xy=annot["xy"], xytext=annot["xytext"], textcoords="data",
                                xycoords="data",
                                arrowprops=dict(facecolor='black', shrink=0.05), verticalalignment="bottom",
                                horizontalalignment=annot["align"], fontsize=annot["fontsize"])
            else:
                ax.annotate(annot["text"], xy=annot["xy"], textcoords="data", xycoords="data",
                            verticalalignment="bottom", horizontalalignment=annot["align"], fontsize=annot["fontsize"])
        # loc=3,ncol=1,bbox_to_anchor=(0.10, 0.70),

        ax.legend(lines,loc=(0.08,0.80),labels=legend_labels1,prop={'size':legend_fontsize})

        #################################################################################
        # axis 2

        lines2=[]
        for index in range(len(stats2)):
            if scatter:
                line = ax2.scatter(x, stats2[index], color=colors[index], marker=markers[index], s=300)
            else:
                xx = x if len(stats2[index]) == len(x) else x[0:len(stats2[index])]

                markers_on = range(0, len(xx), interval_width)
                line, = ax2.plot(xx, stats2[index], color=colors2[index], marker=markers2[index], markevery=markers_on,
                                ms=16, linewidth=4)
            lines2.append(line)

        ax2.legend(lines2,loc=(0.76,0.80), labels=legend_labels2, prop={'size': legend_fontsize})



        # fig.legend(lines, labels=legend_labels, loc=3, ncol=legend_cols, bbox_to_anchor=legendbox,
        #            prop={'size': legend_fontsize})


        fig.tight_layout()
        fig.show()
        if force:
            accept="y"
        else:
            accept = input("accept y/n")
        if accept == "y":
            fig.savefig(save_filename)
            return
        redo = str(input("redo settings ? y/n"))
        if redo == "y":
            ylim[1] = float(input("maxy"))
            ylim[0] = float(input("miny"))
            yscale = str(input("scale: linear/log/symlog/logit"))
            figx = int(input("figx"))
            figy = int(input("figy"))
            legbox_x = float(input("legbox_x"))
            legbox_y = float(input("legbox_y"))
        create_annot = str(input("create/adjust annotation ? y/n"))
        if create_annot == "y":
            added_annot = {}
            added_annot["text"] = str(input("text"))
            num_points = int(input("number of points to annotate"))
            if num_points == 1:
                added_annot["xy"] = tuple(map(float, input("xy").split(',')))
            else:
                added_annot["xy"] = []
                for point in num_points:
                    added_annot["xy"].append(tuple(map(float, input("xy%d" % (point)).split(','))))

            with_arrow = str(input("with_arrow ? y/n"))
            if with_arrow == "y":
                added_annot["xytext"] = tuple(map(float, input("xytext").split(',')))
            annot["align"] = str(input("horizontal alignment"))
            # added_annot["h_align"] = str(input("horizontalalignment"))
            index = int(input("annotation_index"))
            if index + 1 <= len(annotations):
                annotations[index] = added_annot
            else:
                annotations.append(added_annot)

        legendbox = (legbox_x, legbox_y)


def createBoxPlot(numbers,legend_labels,xlabel,ylabel,ylim,save_filename):

    # Create a figure instance
    fig = PLT.figure()
    a = fig.add_subplot(111)

    bp = a.boxplot(numbers)
    #a.scatter(range(1,1+len(methods)), means)
    PLT.xticks(range(1,len(legend_labels)+1), legend_labels, fontsize=8)
    PLT.ylim(ylim)
    PLT.ylabel(ylabel)
    PLT.xlabel(xlabel)
    PLT.subplots_adjust(wspace=0.6, hspace=0.6, left=0.1, bottom=0.22, right=0.96, top=0.96)
    # fig4.tight_layout()
    PLT.tight_layout()
    PLT.savefig(save_filename)



def createContourPlot(z,x,y,xlabel,ylabel,limits,levels,save_filename,xticks=[],yticks=[],title=None):
    """

    :param z: (YxX) --> performance
    :param x:
    :param y:
    :param xlabel:
    :param ylabel:
    :param limits:
    :param levels:
    :param save_filename:
    :param xticks:
    :param yticks:
    :param title:
    :return:
    """




    fig = PLT.figure()
    ax = fig.gca()
    #plt.style.use('seaborn-white')
    X, Y = np.meshgrid(x, y)
    if z is None:
        def fun(x, y):
            return x ** 2 + y
        z = np.array([fun(x, y) for x, y in zip(np.ravel(X), np.ravel(Y))])
        Z = z.reshape(X.shape)
    else:
        Z = z


    #ax.contour(X, Y, Z, levels, cmap=plt.cm.Reds)
    ax.axis(limits)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    #ax.set_zlabel(zlabel)
    if xticks:
        ax.set_xticks(xticks)
    if yticks:
        ax.set_yticks(yticks)
    contourplot = PLT.contourf(X,Y,Z, levels, cmap=PLT.cm.bone,
                               origin='lower')
    cbar = PLT.colorbar(contourplot)
    if title is not None:
        PLT.title(title)
    PLT.show()
    accept = input("accept y/n")
    if accept == "y":
        fig.savefig(save_filename)
        return










def get_colors(num):
    if num < 10:
        cols=['C0', 'C1', 'C2', 'C3', 'C4', 'C5', 'C6', 'C7', 'C8', 'C9']
        return cols[0:num]
    else:
        import matplotlib._color_data as mcd
        cols=[name for name in mcd.CSS4_COLORS if "xkcd:" + name in mcd.XKCD_COLORS]
        xkcd = [mcd.XKCD_COLORS["xkcd:" + str(n)].upper() for n in cols]
        return xkcd
def get_markers(num):
    markers=[".","o","v","D","X","+","x","<",">","d"]
    return markers[0:num]



def heatmap(data, row_labels, col_labels, ax=None,
            cbar_kw={}, cbarlabel="", **kwargs):
    """
    Create a heatmap from a numpy array and two lists of labels.

    Arguments:
        data       : A 2D numpy array of shape (N,M)
        row_labels : A list or array of length N with the labels
                     for the rows
        col_labels : A list or array of length M with the labels
                     for the columns
    Optional arguments:
        ax         : A matplotlib.axes.Axes instance to which the heatmap
                     is plotted. If not provided, use current axes or
                     create a new one.
        cbar_kw    : A dictionary with arguments to
                     :meth:`matplotlib.Figure.colorbar`.
        cbarlabel  : The label for the colorbar
    All other arguments are directly passed on to the imshow call.
    """

    if not ax:
        ax = PLT.gca()

    # Plot the heatmap
    im = ax.imshow(data, **kwargs)

    # Create colorbar
    cbar = ax.figure.colorbar(im, ax=ax, **cbar_kw)
    cbar.ax.set_ylabel(cbarlabel, rotation=-90, va="bottom")

    # We want to show all ticks...
    ax.set_xticks(np.arange(data.shape[1]))
    ax.set_yticks(np.arange(data.shape[0]))
    # ... and label them with the respective list entries.
    ax.set_xticklabels(col_labels)
    ax.set_yticklabels(row_labels)

    # Let the horizontal axes labeling appear on top.
    ax.tick_params(top=True, bottom=False,
                   labeltop=True, labelbottom=False)

    # Rotate the tick labels and set their alignment.
    PLT.setp(ax.get_xticklabels(), rotation=-30, ha="right",
             rotation_mode="anchor")

    # Turn spines off and create white grid.
    for edge, spine in ax.spines.items():
        spine.set_visible(False)

    ax.set_xticks(np.arange(data.shape[1]+1)-.5, minor=True)
    ax.set_yticks(np.arange(data.shape[0]+1)-.5, minor=True)
    ax.grid(which="minor", color="w", linestyle='-', linewidth=3)
    ax.tick_params(which="minor", bottom=False, left=False)

    return im, cbar


def get3DScatter(data,xx,yy,xlabel,ylabel,zlabel):
    from numpy import nan
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x=[]
    y=[]
    z=[]
    for i in xx:
        for j in yy:
            x.append(i)
            y.append(j)
            z.append(data[i][j])

    ax.scatter(x,y,z)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_zlabel(zlabel)