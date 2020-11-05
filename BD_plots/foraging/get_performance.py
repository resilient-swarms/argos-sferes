

from foraging.foraging_params import *
from process_archive_data import *

def index2fullperformance(bd_t,path,faultpath, virtual_folder, best_index, r, gener, tag, estimate):
    # get the corresponding BD
    samp = faultpath + virtual_folder + "/BO_output" + tag + "/samples.dat"
    samp_list = read_spacedelimited(samp)
    sample = None
    i = 0
    for line in samp_list:
        if i == best_index:
            sample = tuple(line[1:])
            break
        i += 1
    # get the corresponding individual in the normal environment, original archive
    original_archive = HOME_DIR + "/Data/ForagingLarge/"+bd_t+ "/results" + str(runs[r]) + "/archive_" + str(gener) + ".dat"
    archive = read_spacedelimited(original_archive)
    archive_individual = None
    for item in archive:
        bd = tuple(item[1:-1])
        if bd == sample:
            archive_individual = item[0]
            break
    if archive_individual is None:
        raise Exception("sample not found in normal archive")


    # if performance is None:
    #     raise Exception("individual not found in faulty archive")
    timefile = faultpath + virtual_folder + "/BO_output" + tag + "/fitness" + str(archive_individual) + ".dat"
    parsed_file_list = read_tabdelimited(timefile)
    time_consumed =float(parsed_file_list[0][2])/TICKS_PER_SECOND #time_consumed = min(float(line[-1]), TICKS_PER_TRIAL)/TICKS_PER_SECOND
    if time_consumed > NUM_SECONDS:
        print(time_consumed)
    if estimate:
        performance = float(parsed_file_list[0][0])
    else:
        # look up that individual's performance in the faulty environment
        parsed_file_list = read_spacedelimited(path)

        performance = 0.0
        for item in parsed_file_list:
            indiv = item[0]
            if indiv == archive_individual:
                performance = float(item[-1])
    return performance, time_consumed

def get_nofault_performances(nofaultpath, gener, runs):

    nofaultfilenames = [nofaultpath + str(run) + "/archive_" + str(gener) + ".dat" for run in runs]
    nofaultperfs = [np.array(list(get_ind_performances_uniquearchive(f).values())).flatten() for f in
                    nofaultfilenames]
    max_nofaultperfs = [max(nofaultperfs[f]) for f in range(len(nofaultfilenames))]
    maxindsnofault = []
    for f in range(len(nofaultfilenames)):
        maxindnofault, best_performance = get_best_individual(nofaultfilenames[f], add_performance=True,
                                                              index_based=False)
        maxindsnofault.append(maxindnofault)
        assert best_performance == max_nofaultperfs[f]
    return nofaultperfs, max_nofaultperfs, maxindsnofault

def get_max_performances(bd_type,fitfuns,generation):
    maximum={fitfun: 0.0 for fitfun in fitfuns}
    for j in range(len(fitfuns)):
        for i in range(len(bd_type)):

            print(fitfuns[j])
            BD_dir = datadir + "/ForagingLarge"
            # get all the data from the archive: no fault

            nofaultpath = BD_dir + "/" + bd_type[i] + "/results"

            nofaultperfs,max_nofaultperfs,maxindsnofault = get_nofault_performances(nofaultpath,generation,runs)
            max_performance = max(max_nofaultperfs)

            if max_performance>maximum[fitfuns[j]]:
                maximum[fitfuns[j]]=max_performance


    pickle.dump(maximum, open("../data/fitfun/foraging_maximal_fitness.pkl", "wb"))

def get_best_performance_VE(path,faultpath,virtual_folder,bd_t,r,gener, until=None,estimate=True,VE_tag=""):
    # look up best observation of virtual energy
    obs = faultpath + virtual_folder + "/BO_output" + VE_tag + "/observations.dat"
    obs_list = read_spacedelimited(obs)
    i = 0
    best_VE = -float("inf")
    best_index = None
    for line in obs_list:
        if i > 0:  # ignore the first line
            number = float(line[-1])
            if number > best_VE:
                best_VE = number
                best_index = i  # ignore the first line
        i +=1
        if i > until:
            break
    return index2fullperformance(bd_t,path,faultpath,virtual_folder,best_index,r,gener, VE_tag,estimate )