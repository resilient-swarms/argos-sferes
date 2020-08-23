from perturbation_analysis import *



def count_nonnormal_solutions(fitfuns, runs):
    """
    count the number of solutions evolved in non-normal environment
    :return:
    """

    # MaxSpeed in 5 10 15 20
    # Robots in 5 10 15 20
    # Wall in 3 4 5 6
    # Cylinder in 0 2 4 6
    # RabRange in 0.25 0.50 1.0 2.0
    # ProxiRange in 0.055 0.11 0.22 0.44

    normal_environment = np.array([0.25,0.25,0.25,0,0.50,0.25])  # 1/4, 1/4, 1/4, 0/4, 2/4,1/4
    count = np.zeros(6)
    for j in range(len(fitfuns)):
        print(fitfuns[j])
        BD_dir = get_bd_dir(fitfuns[j])
        for fault in range(len(faults)):
            for r in runs:

                faultpath = BD_dir + "/environment_diversity/faultyrun" + str(r) + "_p" + str(fault) + "/results" + str(r)+\
                "/analysis" + str(generation) + "_handcrafted.dat"
                nofaultpath =  BD_dir + "/environment_diversity/results" + str(r)+"/archive_"+str(generation)+".dat"
                # get the best recovery solution and its environment descriptor
                maxind, best_performance = get_best_individual(faultpath, add_performance=True, index_based=True)
                parsed_file_list = read_spacedelimited(nofaultpath)
                bd = parsed_file_list[maxind][1:-1]
                for i in range(len(count)):
                    if float(bd[i]) != normal_environment[i]:
                        count[i]+=1


    print("count =",count)

def count_nonnormal_solutions(fitfuns, runs):
    """
    count the number of solutions evolved in non-normal environment
    :return:
    """

    # MaxSpeed in 5 10 15 20
    # Robots in 5 10 15 20
    # Wall in 3 4 5 6
    # Cylinder in 0 2 4 6
    # RabRange in 0.25 0.50 1.0 2.0
    # ProxiRange in 0.055 0.11 0.22 0.44

    normal_environment = np.array([0.25,0.25,0.25,0,0.50,0.25])  # 1/4, 1/4, 1/4, 0/4, 2/4,1/4
    count = np.zeros(6)
    for j in range(len(fitfuns)):
        print(fitfuns[j])
        BD_dir = get_bd_dir(fitfuns[j])
        for fault in range(len(faults)):
            for r in runs:

                faultpath = BD_dir + "/environment_diversity/faultyrun" + str(r) + "_p" + str(fault) + "/results" + str(r)+\
                "/analysis" + str(generation) + "_handcrafted.dat"
                nofaultpath =  BD_dir + "/environment_diversity/results" + str(r)+"/archive_"+str(generation)+".dat"
                # get the best recovery solution and its environment descriptor
                maxind, best_performance = get_best_individual(faultpath, add_performance=True, index_based=True)
                parsed_file_list = read_spacedelimited(nofaultpath)
                bd = parsed_file_list[maxind][1:-1]
                for i in range(len(count)):
                    if float(bd[i]) != normal_environment[i]:
                        count[i]+=1


    print("count =",count)


def count_nonnormal_solutions(fitfuns, runs):
    """
    count the number of solutions evolved in non-normal environment
    :return:
    """

    # MaxSpeed in 5 10 15 20
    # Robots in 5 10 15 20
    # Wall in 3 4 5 6
    # Cylinder in 0 2 4 6
    # RabRange in 0.25 0.50 1.0 2.0
    # ProxiRange in 0.055 0.11 0.22 0.44

    normal_environment = np.array([0.25,0.25,0.50,0,0.50,0.25])  # 1/4, 1/4, 2/4, 0/4, 2/4,1/4
    nonnormalcount = np.zeros(6)
    nonnormalresilience = [[] for i in range(6)]
    count = np.zeros(shape=(6,4))
    resilience = [[[] for i in range(4)] for j in range(6)]
    total_resilience = []
    np.set_printoptions(precision=3)
    for j in range(len(fitfuns)):
        print(fitfuns[j])
        BD_dir = get_bd_dir(fitfuns[j])
        for fault in range(len(faults)):
            for r in runs:

                faultpath = BD_dir + "/environment_diversity/faultyrun" + str(r) + "_p" + str(fault) + "/results" + str(r)+\
                "/analysis" + str(generation) + "_handcrafted.dat"
                bdpath =  BD_dir + "/environment_diversity/results" + str(r)+"/archive_"+str(generation)+".dat"
                nofaultpath =  BD_dir + "/environment_diversity/FAULT_NONE/results" + str(r)+"/analysis" + str(generation) + "_handcrafted.dat"

                # get the best recovery solution and its environment descriptor
                maxind, best_performance = get_best_individual(faultpath, add_performance=True, index_based=True)
                parsed_file_list = read_spacedelimited(bdpath)
                bd = np.array(parsed_file_list[maxind][1:-1],dtype=float)
                bin = np.array(bd * 4.0, dtype=int)
                maxind_norm, best_performance_norm = get_best_individual(nofaultpath, add_performance=True,
                                                                         index_based=True)
                res = (best_performance - best_performance_norm)/ best_performance_norm
                for i in range(len(count)):
                    if bd[i] != normal_environment[i]:
                        nonnormalcount[i]+=1
                        nonnormalresilience[i] = np.append(nonnormalresilience[i],res)
                    count[i,bin[i]]+=1
                    resilience[i][bin[i]] = np.append(resilience[i][bin[i]],res)

    for i in range(len(count)):
        for j in range(len(count[0])):
            resilience[i][j] = np.median(resilience[i][j])
        nonnormalresilience[i] = np.median(nonnormalresilience[i])
        print(np.array(resilience[i]))
    print("nonnormalcount =",nonnormalcount)
    print("count =", count)

    print("nonnormalresilience ",np.array(nonnormalresilience))
    print("median resilience", np.median(resilience))
    print("sum =",np.sum(count)/6.0)

if __name__ == "__main__":
    count_nonnormal_solutions(fitfuns, runs)


