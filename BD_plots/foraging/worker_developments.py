


from foraging.get_performance import *


def get_BO_development(bd_t, r, gener, path, faultpath, best_performances,time_lost, normal_folder,virtual_folder,virtual_energy,uniform,estimate):

    if virtual_energy:
        BOfile = faultpath + virtual_folder +"/BO_output"+VE_tag+"/best_observations.dat"
    elif uniform:
        BOfile = faultpath + normal_folder + "/BO_output" + uniform_tag + "/best_observations.dat"
    else:
        BOfile = faultpath + normal_folder + "/BO_output" + settings_tag + "/best_observations.dat"
    parsed_file_list = read_spacedelimited(BOfile)

    i=1
    time_cumulant=0.0
    x=[]
    y=[]
    count_full_eval=0
    for line in parsed_file_list:
        if i==1:
            i+=1
            continue # ignore the first line
        if virtual_energy:
            best_performance,time_consumed=get_best_performance_VE(path,faultpath,virtual_folder,bd_t,r,gener,until=i,estimate=estimate)
        else:
            best_performance=float(line[-1])
            time_consumed=NUM_SECONDS
        time_cumulant+=time_consumed
        x.append(time_cumulant)
        y.append(best_performance)
        i+=1
    time_lost = np.append(time_lost, x)
    best_performances = np.append(best_performances, y)
    return best_performances,  time_lost

def get_baseline_development(faultpath, title_tag, best_performances,time_lost):
    BOfile = faultpath + "/baselines/"+ title_tag + "/" + title_tag
    parsed_file_list = read_spacedelimited(BOfile)

    i=1
    time_cumulant=0.0
    x=[]
    y=[]
    count_full_eval=0
    best_performance=-float("inf")
    for line in parsed_file_list:
        performance=float(line[-1])
        if performance > best_performance:
            best_performance=performance
        time_consumed=NUM_SECONDS
        time_cumulant+=time_consumed
        x.append(time_cumulant)
        y.append(best_performance)
        i+=1
    time_lost = np.append(time_lost, x)
    best_performances = np.append(best_performances, y)
    return best_performances,  time_lost
def get_worker_developments(num_evals,num_workers,faultpath, normal_folder,VE_tag):

    mean_time=[[] for i in range(num_evals)]
    mean_y=[[] for i in range(num_evals)]
    for worker in range(num_workers):
        BOfile = faultpath + normal_folder + "/BO_output" + VE_tag + "/async_stats_best"+str(worker)+".dat"
        parsed_file_list = read_spacedelimited(BOfile)
        x=[]
        y=[]
        i=0
        for line in parsed_file_list:
            print(i)
            best_performance=float(line[-1])
            time_cumulant=float(line[0])/(NUM_TRIALS*TICKS_PER_SECOND)
            mean_time[i].append(time_cumulant)
            mean_y[i].append(best_performance*NUM_AGENTS/num_workers)
            i+=1
            if i==num_evals:
                break
    return mean_y,  mean_time

def get_workergroup_developments(num_evals,num_workers,path):
    """
    assuming synchronous distributed
    :param num_evals:
    :param num_workers:
    :param faultpath:
    :param normal_folder:
    :return:
    """

    mean_time=[[] for i in range(num_evals)]
    mean_y=[[] for i in range(num_evals)]
    past_workers = 0
    for worker in range(num_workers):
        if past_workers == num_workers:
            break # finished all workers
        BOfile = path + "/async_stats_best"+str(worker)+".dat"
        parsed_file_list = read_spacedelimited(BOfile)
        time_init=float(parsed_file_list[0][0])
        group_size=1
        for line in parsed_file_list[1:]:
            time = float(line[0])
            if time == time_init:
                group_size+=1
            else:
                break
        past_workers+=group_size

        i = 0
        for line in parsed_file_list:
            print(i)
            index = i // group_size
            if i % group_size == 0:
                best_performance=float(line[-1])
                time_cumulant = float(line[0]) / (NUM_TRIALS * TICKS_PER_SECOND)
            else:
                performance = float(line[-1])
                if performance > best_performance:
                    best_performance=performance
            if i % group_size == group_size - 1:
                mean_time[index]=np.append(mean_time[index],[time_cumulant for j in range(group_size)])
                mean_y[index]=np.append(mean_y[index],[best_performance * NUM_AGENTS / num_workers for j in range(group_size)])
            i+=1
            if i==num_evals*group_size:
                break
    return mean_y,  mean_time

