
import argparse
from process_archive_data import *


parser = argparse.ArgumentParser(description='Process arguments.')
parser.add_argument('-i', type=str,help='input archive path, with all the translated individuals')
parser.add_argument('-o', type=str, help="output archive path, with translated invididuals reduced to calculate the correct coverage" )
args = parser.parse_args()


def transform_bd_mapelites(bd,behav_shape):
    behav = []
    for i in range(len(bd)):
        b = round(bd[i] * behav_shape[i])
        behav.append(min(b, behav_shape[i] - 1) / float(behav_shape[i]))
    return tuple(behav)
def transform_bd_cvtmapelites(bd,centroids):
    min_dist = float("inf")

    for centroid in centroids:
        dist = calc_dist(centroid, bd);
        if (dist < min_dist):
            min_dist = dist
            min_centr = centroid
    return tuple(min_centr)
def transform(bd,helper,type="mapelites"):
    if type=="mapelites":
        return transform_bd_mapelites(bd, helper)
    else:
        return transform_bd_cvtmapelites(bd, helper)

def reduce_translated_archive(archive_file,bd_condition,new_archive_file,helper_data):
    """
    from a translated archive, reduce it such that it only contains the best for each bin
    :return:
    """
    new_archive={}
    archive,individuals=get_bin_performances(archive_file,as_string=False, add_indiv=True,fitnessfile=False)
    i=0
    for bd,fitness in archive.items():
        if bd_condition(bd,fitness,new_archive):
            bd=np.array(bd,dtype=float)
            descriptor=transform(bd,helper_data)
            individual=individuals[i]
            new_archive[descriptor] = (individual,fitness)
        i+=1

    # finally print the new archive to file
    f=open(new_archive_file,"w")
    for bd, entry in new_archive.items():
        individual,fitness=entry
        f.write(individual+  "   ")
        for i in range(len(bd)):
            f.write(str(bd[i]) + "   ")
        f.write(str(fitness) + "\n")
        f.flush()
    f.close()

def mapelites_bd_condition(behav,fitness,new_archive,epsilon=0.0):

    existing_entry = new_archive.get(tuple(behav))
    return existing_entry is None or fitness > existing_entry + epsilon # remove additional condition because distance to center is no longer relevant (no more evolution)
    # behav_pos=[]
    # for i in range(len(bd)):
    #     behav_pos[i] = round(bd[i] * behav_shape[i]);
    # behav_pos[i] = std::
    #     min(behav_pos[i], behav_shape[i] - 1);
    # assert (behav_pos[i] < behav_shape[i]);
    #


def calc_dist(p1,p2):
    dist = 0.0
    for i in range(len(p1)):
        dist += pow(p1[i] - p2[i], 2);

    return dist**0.5


def cvt_mapelites_bd_condition(bd, fitness,new_archive, centroids):
    min_dist = float("inf")
    for centroid in centroids:
        dist = calc_dist(bd,centroid)

        if (dist < min_dist):
            min_dist = dist
            archive_index = centroid
    existing_entry = new_archive.get(bd)
    if existing_entry is None or fitness > existing_entry:
            return True

    return False


if __name__ == "__main__":
    for fitfun in ["Aggregation","Dispersion"]:
        for gen in range(0,2500,500):
            input="/home/david/Data/ExperimentData/"+fitfun+"range11/environment_diversity/FAULT_NONE/results1/analysis"+str(gen)+"_handcrafted.dat"
            output="/home/david/Data/ExperimentData/"+fitfun+"range11/environment_diversity/FAULT_NONE/results1/analysis"+str(gen)+"_handcraftedREDUCED.dat"
            reduce_translated_archive(args.i,mapelites_bd_condition,args.o,helper_data=(16,16,16))