
from foraging.foraging_params import *
from significance import *
from plots import *


def make_significance_table(fitfunlabels,conditionlabels,qed_index,table_type="resilience"):

    best_performance_data, performance_data, best_transfer_data, transfer_data, resilience_data = pickle.load(
            open("../data/fitfun/summary_statistics_fitfun.pkl", "rb"))
    if table_type=="resilience":
        qed=resilience_data[qed_index] # QED
        data=resilience_data
    else:
        for i, fitfun in enumerate(fitfunlabels):
            for j in range(len(best_performance_data)):
                best_performance_data[j][i]/=baseline_performances[fitfun]
        qed=best_performance_data[qed_index]
        data=best_performance_data
    with open("results/fault/table/significance_table"+table_type,"w") as f:
        f.write(r"& \multicolumn{9}{c}{\textbf{Condition}}")
        newline_latex(f,add_hline=True)
        f.write(r"\textbf{Swarm task}")
        f.write(r"& QED")
        for condition in conditionlabels:
            if condition != "QED":
                f.write(r"& \multicolumn{3}{c|}{"+str(condition)+"}")
        newline_latex(f,add_hline=True)
        f.write(r"& %s " % (table_type))
        for condition in conditionlabels:
            if condition != "QED":
                f.write(r"& %s & significance & effect"%(table_type))
        newline_latex(f,add_hline=True)

        m=len(fitfuns)*3 # number of comparisons
        alpha_weak=.05/float(m)
        print("will use alpha=" + str(alpha_weak))
        alpha_best=.001/float(m) #
        print("will use alpha="+str(alpha_best))
        for k,fitfun in enumerate(fitfunlabels):
            f.write(fitfun)
            x = qed[k]
            # first write QED's resilience score
            mx = np.median(x)
            iqrx = np.quantile(x, 0.75) - np.quantile(x, 0.25)
            f.write(r"& $%.2f \pm %.1f$" % (mx,iqrx))
            # now write other's resilience score and significance values
            for j, condition in enumerate(conditionlabels):
                if condition == "QED":
                    continue
                y = data[j][k]
                m=np.median(y)
                iqr=np.quantile(y,0.75) - np.quantile(y,0.25)
                U, p = ranksums(x, y)
                p_value = "p=%.3f"%(p) if p>0.001 else r"p<0.001"
                if p < alpha_best:
                    p_value+="^{**}"
                else:
                    if p < alpha_weak:
                        p_value+="^{*}"
                delta,label = cliffs_delta(U, x, y)
                delta_value = r"\mathbf{%.2f}"%(delta) if label == "large" else r"%.2f"%(delta)
                f.write(r"& $%.2f \pm %.1f$ & $%s$ & $%s$"%(m,iqr,p_value,delta_value))
            newline_latex(f)

def make_significance_table_compact(fitfunlabels,conditionlabels,qed_index,table_type="resilience"):

    best_performance_data, performance_data, best_transfer_data, transfer_data, resilience_data = pickle.load(
            open("../data/fitfun/summary_statistics_fitfun.pkl", "rb"))
    if table_type=="resilience":
        qed=resilience_data[qed_index] # QED
        data=resilience_data
    else:
        for i, fitfun in enumerate(fitfunlabels):
            for j in range(len(best_performance_data)):
                best_performance_data[j][i]/=baseline_performances[fitfun]
        qed=best_performance_data[qed_index]
        data=best_performance_data
    with open("results/fault/table/significance_table_compact"+table_type,"w") as f:
        f.write(r"& \multicolumn{6}{c}{\textbf{Condition}}")
        newline_latex(f,add_hline=True)
        f.write(r"\textbf{Swarm task}")
        f.write(r"& QED")
        for condition in conditionlabels:
            if condition != "QED":
                f.write(r"& \multicolumn{2}{c|}{"+str(condition)+"}")
        newline_latex(f,add_hline=True)
        f.write(r"& %s " % (table_type))
        for condition in conditionlabels:
            if condition != "QED":
                f.write(r"& effect & significance "%(table_type))
        newline_latex(f,add_hline=True)

        m=len(fitfuns)*3 # number of comparisons
        alpha_weak=.05/float(m)
        print("will use alpha=" + str(alpha_weak))
        alpha_best=.001/float(m) #
        print("will use alpha="+str(alpha_best))
        for k,fitfun in enumerate(fitfunlabels):
            f.write(fitfun)
            x = qed[k]
            # now write other's resilience score and significance values
            for j, condition in enumerate(conditionlabels):
                if condition == "QED":
                    continue
                y = data[j][k]
                U, p = ranksums(x, y)
                p_value = "p=%.3f"%(p) if p>0.001 else r"p<0.001"
                if p < alpha_best:
                    p_value+="^{**}"
                else:
                    if p < alpha_weak:
                        p_value+="^{*}"
                delta,label = cliffs_delta(U, x, y)
                delta_value = r"\mathbf{%.2f}"%(delta) if label == "large" else r"%.2f"%(delta)
                f.write(r" & $%s$ & $%s$"%(delta_value,p_value))
            newline_latex(f)
def write_conditional(performance_list,index,file,max_reference,min_reference):
    U, p = ranksums(performance_list[0],performance_list[index])
    m_temp=np.mean(performance_list[index])
    sd_temp=np.std(performance_list[index])
    if p < 0.05:
        if U > 0 :
            file.write("$\mathbf{%.2f \pm %.2f}$ (+) "%(m_temp, sd_temp))
        else:
            file.write("$\mathbf{%.2f \pm %.2f}$ (-) " % (m_temp, sd_temp))
    else:
        if U > 0:
            file.write("$%.2f \pm %.2f$ " % (m_temp, sd_temp))
        else:
            file.write("$%.2f \pm %.2f$ " % (m_temp, sd_temp))
    if index==0:
        ref=100*(m_temp/min_reference) - 100
        file.write(" $(%.1f%%)$ &"%(ref))
    else:
        #ref=100*(m_temp/min_reference)
        file.write(" &")


def make_simple_table(best_performance_data,time_loss,max_evals,conditions, plottag, num_fault_categories):
    # get the mean and sd for each fault category for this condition (BO or BO-VE)
    bd_index=0
    table30_file = open("performance_table"+plottag+"30.txt","w")
    for fault_category in range(num_fault_categories):
        performances30 = [None for c in conditions]
        table30_file.write(foraging_fault_types[fault_category] + " & ")
        time = [[] for c in conditions]


        for c, condition in enumerate(conditions):
            # after equivalent of 30 evals
            t_3600 = None
            p_3600 = None
            p_sd_3600 = None
            mindist_3600 = float("inf")

            for t in range(max_evals[c]):
                data = best_performance_data[c][fault_category][t]/NUM_AGENTS
                mean = np.mean(data)
                sd = np.std(data)
                consumed = np.mean(time_loss[c][fault_category][t])

                if consumed >=29*NUM_SECONDS and consumed <=31*NUM_SECONDS: # try to find closest to 360
                    dist = abs(consumed - 30*NUM_SECONDS)
                    if dist < mindist_3600:
                        t_3600 = consumed
                        p_3600 = mean
                        p_sd_3600 = sd
                        mindist_3600 = dist
                        performances30[c] = data
            # print(str(t_1200) + " " + str(p_1200) + " " + str(p_sd_1200))
            # print(str(t_2400) + " " + str(p_2400) + " " + str(p_sd_2400))
            print(str(t_3600) + " " + str(p_3600) + " " + str(p_sd_3600))
            if c==len(conditions) - 1:
                table30_file.write(r"  $%.3f \pm %.2f$ \\\\"%(np.mean(performances30[c]),np.std(performances30[c])))
            else:
                table30_file.write(r"  $%.3f \pm %.2f$ &")
        table30_file.write("\n")