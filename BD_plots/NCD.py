import bz2
import tarfile
from process_archive_data import *
import pickle

def get_help_data(directory,history_type,runs):
    max_performance = -float("inf")
    max_run = None
    max_indiv = None
    for run in runs:
        file = directory+str(run)+"/analysis_sdbc.dat"
        best_indiv, performance = get_best_individual(file,add_performance=True)
        if performance > max_performance:
            max_performance = performance
            max_run = run
            max_indiv = best_indiv
    history_file = directory + str(max_run) + "/" + history_type + "_history" + str(max_indiv)
    return history_file, max_performance
def gather_NCDs(BD_DIRECTORY,faults, runs, history_type="sa"):
    """
    use existing state/observation-action trajectory files and calculate pair-wise NCDs for all individuals within the sa;e run
    the average is returned
    :return:
    """
    assert faults[0]=="FAULT_NONE"

    directories=[ BD_DIRECTORY+"/"+faults[f] + "/results" for f in range(len(faults)) ]
    ncds = []
    delta_ps=[]
    history_comp, performance_comp = get_help_data(directories[0],history_type,runs)

    for i in range(1,len(directories)):
        history_file, performance = get_help_data(directories[i], history_type, runs)
        # get delta_p
        delta_p = performance - performance_comp
        print("performance difference: " +str(delta_p))
        delta_ps.append(delta_p)
        # get ncd
        ncd = NCD(history_comp,history_file)
        ncds.append(ncd)



    return ncds, delta_ps

def read_history_file(filename,from_gz=True):
    if from_gz:
        print("opening "+str(filename)+".tar.gz")
        tar = tarfile.open(filename+".tar.gz")
        member = tar.getmembers()[0]  # assumes only a single file in the archive
        f = tar.extractfile(member)
        content=f.read()
        tar.close()
        return content
    else:
        x = open(filename, 'rb').read()  # file 1
    return x

def test_NCD(compressor,num_agents, num_trials,num_ticks, num_features):
    """
    test NCD on history files by :
        comparing two identical files
        comparing two completely different random files

    :return:
    """

    # first emulate the c++ code to generate histories
    def start_trial(file,trial):
        file.write("T"+str(trial)+"\n")
    def write_input(file,inputs):
        for i in range(len(inputs)):
            file.write("%.1f,"%(inputs[i]))
        file.write("#")

    def create_random_history(filename,num_agents,num_trials,num_ticks,num_features):
        file = open(filename,"w+",1)
        data = np.random.random(size=(num_trials,num_ticks,num_agents,num_features))
        for trial in range(num_trials):
            start_trial(file,trial)
            for tick in range(num_ticks):
                for agent in range(num_agents):
                    write_input(file,data[trial][tick][agent])
                file.write("\n")
            file.write("\n")
        file.flush()
        file.close()
    def create_predictable_history(filename,num_agents,num_trials,num_ticks,num_features):
        file = open(filename,"w+",1)
        for trial in range(num_trials):
            start_trial(file,trial)
            data=np.zeros((num_agents,num_features)) #initialise randomly
            for tick in range(num_ticks):
                for agent in range(num_agents):
                    write_input(file,data[agent])
                file.write("\n")
                data+=0.05
            file.write("\n")
        file.flush()
        file.close()
    # first create random histories

    filename = "dummyhistory1"
    create_random_history(filename,num_agents,num_trials,num_ticks,num_features)
    filename2 = "dummyhistory2"
    create_random_history(filename2, num_agents, num_trials, num_ticks, num_features)


    # pickle.dump(same_bytes,open("dummyhistorycombXX.pkl","wb"))
    # pickle.dump(new_bytes, open("dummyhistorycombXY.pkl","wb"))
    # comparing identical files : NCD ?= 0
    NCD(filename,filename,compressor)
    # comparing two random files : NCD ?= 1
    NCD(filename, filename2,compressor)

    # now create one predictable history
    filename3="dummyhistory3"
    create_predictable_history(filename3,num_agents, num_trials, num_ticks, num_features)

    NCD(filename3, filename3,compressor)
    # comparing two random files : NCD ?= 1
    NCD(filename, filename3,compressor)

    NCD(filename2, filename3, compressor)

# def save_as_string(filename):
#     with open(filename, 'r') as file:
#         data = file.read().replace('\n', '')
#         pickle.dump(data,open(filename+".pkl","wb"))
#
#
def combine_files(filename,filename2):
    x = open(filename,"rb").read()
    y = open(filename2,"rb").read()
    x_y = x + y
    with open("temp","wb",1) as f:
        f.write(x_y)
    return
#     stri = pickle.load(open(filename + ".pkl", "rb"))
#     str2 = pickle.load(open(filename2 + ".pkl", "rb"))
#     new_str = stri + str2
#     pickle.dump(new_str,open("temp.pkl","wb"))

def perform_ppm(file):
    os.system("7z a -mm=PPMd -mmem=256M -mx=9 -mo=32 " + file + ".zip " + file)
    return os.path.getsize(file + ".zip")

def perform_lzma(file):
    os.system("7z a -mm=LZMA -mx=9 " + file + ".zip " + file)
    return os.path.getsize(file + ".zip")
def NCD(file1,file2,compressor):

    # two choicesare important :
    # 1. use the PPM compression
    # Vitányi, P. M. B., Balbach, F. J., Cilibrasi, R. L., & Li, M. (2009). Normalized information distance.
    # Information Theory and Statistical Learning, 45–82. https://doi.org/10.1007/978-0-387-84816-7_3
    # maximum useable length of the arguments x and y =32KB for gzip, 450 KB for bzip2, unlimited for PPMZ

    # also : Alfonseca, M., Cebrián, M., & Ortega, A. (2005). Common Pitfalls Using the Normalized Compression Distance:
    # What to Watch Out for in a Compressor. Communications in Information and Systems, 5(4), 367–384. https://doi.org/10.4310/cis.2005.v5.n4.a1

    # 2. use highest compression level = 9

    print("getting NCDs:")

    import os

    l_x = compressor(file1)
    if file1 != file2:
        l_y = compressor(file2)
    else:
        l_y = l_x
    combine_files(file1, file2)
    l_xy = compressor("temp")
    #os.system("rm temp.zip")
    enum = l_xy - min(l_x, l_y)
    denom = max(l_x, l_y)
    NCD = enum / denom  # NOTE: type case to float not necessary; python 3 uses // for integer division
    print("NCD =%.3f" % (NCD))
    # l_x_c = len(x_c)
    # l_y_c = len(y_c)
    # l_xy_c=len(x_y_c)
    # print(l_x_c)
    # print(l_y_c)
    # print(l_xy_c)
    # enum = l_xy_c - min(l_x_c, l_y_c)
    # denom = max(l_x_c, l_y_c)
    # NCD =  enum / denom  # NOTE: type case to float not necessary; python 3 uses // for integer division
    # print("NCD =%.3f"%(NCD))

# def NCD(file1,file2, from_gz=True):
#
#     print("getting NCDs:")
#     print(file1)
#     x=read_history_file(file1,from_gz)
#     print(file2)
#     y=read_history_file(file2,from_gz)
#
#     x_y = x + y  # concatenation
#
#     x_c = bz2.compress(x)
#     y_c = bz2.compress(y)
#     x_y_c = bz2.compress(x_y)
#
#     l_x_c = len(x_c)
#     l_y_c = len(y_c)
#     l_xy_c=len(x_y_c)
#     print(l_x_c)
#     print(l_y_c)
#     print(l_xy_c)
#     enum = l_xy_c - min(l_x_c, l_y_c)
#     denom = max(l_x_c, l_y_c)
#     NCD =  enum / denom  # NOTE: type case to float not necessary; python 3 uses // for integer division
#     print("NCD =%.3f"%(NCD))

if __name__ == "__main__":
    test_NCD(compressor=perform_lzma,num_agents=10, num_trials=5,num_ticks=2000, num_features=2)