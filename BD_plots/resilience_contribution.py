
from perturbation_analysis import *
HOME_DIR="/home/david"
import operator


def gather_individual_data(gener,runs):
        QED_index=3
        diff_resilience_sorted={}
        impacts_sorted={}
        for j in range(len(fitfuns)):
            print(fitfuns[j])
            BD_dir = get_bd_dir(fitfuns[j])
            # get all the data from the archive: no fault


            # join all the data from all the fault archives:
            differences={}
            impacts={}
            for fault in range(len(faults)):
                print("fault %d" % (fault))
                for r, run in enumerate(runs):

                    performances = []
                    best_performances = []
                    recovery = []
                    resilience = []
                    transfer = []
                    best_transfer = []
                    best_faultperfs = []
                    for i in range(len(bd_type)):
                        print(bd_type[i])
                        nofaultpath = BD_dir + "/" + bd_type[i] + "/FAULT_NONE/results"
                        if bd_type[i] == "baseline":
                            best_nofaultperfs = get_baseline_performances(nofaultpath)
                            nofaultperfs = None
                            maxindsnofault = None
                        else:
                            nofaultperfs, best_nofaultperfs, maxindsnofault = get_nofault_performances(nofaultpath,
                                                                                                       gener, runs)
                        faultpath = BD_dir + "/" + bd_type[i] + "/faultyrun" + str(run) + "_p" + str(
                            fault) + "/results" + str(
                            run)
                        best_performances, performances, best_transfer, transfer, recovery, resilience, best_faultperf = add_fault_performance(
                            j, r, gener, nofaultperfs, best_nofaultperfs, maxindsnofault, faultpath,
                            best_performances, performances, best_transfer, transfer, recovery,
                            resilience, baseline=bd_type[i] == "baseline",add_faultperf=True)
                        best_faultperfs.append(best_faultperf)  # performance of normal individual


                    diff_resilience = (resilience[QED_index] - np.max(resilience[1:QED_index]), best_performances/baseline_performances[fitfuns[j]],
                                      best_faultperfs)  # look at cases that contribute most to resilience advantage
                    differences[(run,fault)]=diff_resilience

            diff_resilience_sorted[fitfuns[j]] = sorted(differences.items(), key=operator.itemgetter(1),reverse=True)


        pickle.dump(diff_resilience_sorted,open("data/fitfun/resilience_differences.pkl","wb"))


def gather_impact_data(gener, runs):

    impact_sorted = {}
    for j in range(len(fitfuns)):
        print(fitfuns[j])
        BD_dir = get_bd_dir(fitfuns[j])
        # get all the data from the archive: no fault

        # join all the data from all the fault archives:
        differences = {}
        impacts = {}
        for fault in range(len(faults)):
            print("fault %d" % (fault))
            for r, run in enumerate(runs):

                performances = []
                best_performances = []
                recovery = []
                resilience = []
                transfer = []
                best_transfer = []
                best_faultperfs = []
                for i in range(len(bd_type)):
                    print(bd_type[i])
                    nofaultpath = BD_dir + "/" + bd_type[i] + "/FAULT_NONE/results"
                    if bd_type[i] == "baseline":
                        best_nofaultperfs = get_baseline_performances(nofaultpath)
                        nofaultperfs = None
                        maxindsnofault = None
                    else:
                        nofaultperfs, best_nofaultperfs, maxindsnofault = get_nofault_performances(nofaultpath,
                                                                                                   gener, runs)
                    faultpath = BD_dir + "/" + bd_type[i] + "/faultyrun" + str(run) + "_p" + str(
                        fault) + "/results" + str(
                        run)
                    best_performances, performances, best_transfer, transfer, recovery, resilience, best_faultperf = add_fault_performance(
                        j, r, gener, nofaultperfs, best_nofaultperfs, maxindsnofault, faultpath,
                        best_performances, performances, best_transfer, transfer, recovery,
                        resilience, baseline=bd_type[i] == "baseline", add_faultperf=True)
                    best_faultperfs.append(best_faultperf)  # performance of normal individual

                diff_resilience = (resilience[QED_index] - np.max(resilience[1:QED_index]),
                                   best_performances / baseline_performances[fitfuns[j]],
                                   best_faultperfs)  # look at cases that contribute most to resilience advantage
                differences[(run, fault)] = diff_resilience

        diff_resilience_sorted[fitfuns[j]] = sorted(differences.items(), key=operator.itemgetter(1), reverse=True)

    pickle.dump(diff_resilience_sorted, open("data/fitfun/resilience_differences.pkl", "wb"))




if __name__ == "__main__":

    # note 2:2 flocking swirling behaviour
    #gather_individual_data(gener=generation,runs=runs)
    diff_res = pickle.load(open("data/fitfun/resilience_differences.pkl","rb"))
    print()

