
import argparse
from process_archive_data import *


parser = argparse.ArgumentParser(description='Process arguments.')
parser.add_argument('-i', type=str,help='input archive path, with all the translated individuals')
parser.add_argument('-o', type=str, help="output archive path, with translated invididuals reduced to calculate the correct coverage" )
args = parser.parse_args()


def transform_bd_mapelites(bd,behav_shape):
    behav = []
    for i in range(len(bd)):
        b = int(bd[i] * behav_shape[i])
        behav.append(min(b, behav_shape[i] - 1) / float(behav_shape[i]))
    return tuple(behav)
def transform_bd_cvtmapelites(bd,centroids):
    min_dist = float("inf")
    min_index = None
    for i in range(len(centroids)):
        dist = calc_dist(centroids[i], bd)
        if (dist < min_dist):
            min_dist = dist
            min_centr = centroids[i]
            min_index = i
    return min_index,tuple(min_centr)


def add_to_archive(individual,bd , fitness,new_archive, transformation,transformdata):

    bin=transformation(bd, transformdata)
    if bin is not None:
        new_archive[bin] = (individual, fitness)

def reduce_translated_archive(archive_file,transform_condition,new_archive_file,transform_data):
    """
    from a translated archive, reduce it such that it only contains the best for each bin
    :return:
    """
    new_archive=get_bin_performances_duplicatearchive(archive_file,transform_condition,transform_data,as_string=False, add_indiv=True)
    # finally print the new archive to file
    f=open(new_archive_file,"w")
    for bd, entry in new_archive.items():
        fitness, individual=entry
        f.write(individual+  "   ")
        if isinstance(bd,int):
            f.write(str(bd) + "   ") # write the index of the centroid
            bd = transform_data[bd] #use the index of the centroid
        for i in range(len(bd)):
            f.write(str(bd[i]) + "   ")
        f.write(str(fitness) + "\n")
        f.flush()
    f.close()

def mapelites_bd_add(behav,behav_shape,new_entry,new_archive,epsilon=0.0):
    behav = np.array(behav, dtype=float)
    behav = transform_bd_mapelites(behav,behav_shape)

    existing_entry = new_archive.get(behav)
    if existing_entry is None:
        new_archive[behav] = new_entry
        return
    new_fitness, _new_ind = new_entry
    old_fitness, _individual = existing_entry
    if new_fitness > old_fitness + epsilon:# remove additional condition because distance to center is no longer relevant (no more evolution)
        new_archive[behav] = new_entry

    #    new_archive[behav] = (individual, fitness)
    # behav_pos=[]
    # for i in range(len(bd)):
    #     behav_pos[i] = round(bd[i] * behav_shape[i]);
    # behav_pos[i] = std::
    #     min(behav_pos[i], behav_shape[i] - 1);
    # assert (behav_pos[i] < behav_shape[i]);
    #


def calc_dist(p1,p2):

    dist = np.sum(np.square(p1 - p2))

    return dist**0.5


def cvt_mapelites_bd_add(bd, centroids, new_entry,new_archive):
    behav = np.array(bd, dtype=float)
    index, behav = transform_bd_cvtmapelites(behav, centroids)
    existing_entry = new_archive.get(index)
    if existing_entry is None:
        new_archive[index] = new_entry
        return
    old_fitness, _individual = existing_entry
    new_fitness, _new_indiv = new_entry
    if new_fitness > old_fitness:
        new_archive[index] = new_entry
    #  new_archive[behav] = (individual, fitness)


def test_cases():
    """ test some simple cases"""
    new_archive = {}
    mapelites_bd_add(("0.15", "0.0"), (10, 10), (0.8, 0), new_archive)
    mapelites_bd_add(("0.15", "0.0"), (10, 10), (0.9, 0), new_archive)
    mapelites_bd_add(("0.38", "0.0"), (10, 10), (0.5, 0), new_archive)
    mapelites_bd_add(("0.38", "0.0"), (10, 10), (0.8, 0), new_archive)

    print(new_archive)

    new_archive={}
    centroids=[np.array([0.0,0.0]),np.array([1.0,1.0])]
    cvt_mapelites_bd_add(("0.15", "0.0"), centroids, (0.8, 0), new_archive)
    cvt_mapelites_bd_add(("0.15", "0.0"), centroids, (0.9, 0), new_archive)
    cvt_mapelites_bd_add(("0.38", "0.0"), centroids, (0.5, 0), new_archive)
    cvt_mapelites_bd_add(("0.38", "0.0"), centroids, (0.8, 0), new_archive)

    cvt_mapelites_bd_add(("0.55", "0.55"), centroids, (0.8, 0), new_archive)
    cvt_mapelites_bd_add(("0.55", "0.55"), centroids, (0.95, 0), new_archive)
    cvt_mapelites_bd_add(("0.55", "0.55"), centroids, (0.5, 0), new_archive)
    cvt_mapelites_bd_add(("0.555", "0.55"), centroids, (0.8, 0), new_archive)

    print(new_archive)



if __name__ == "__main__":
    #test_cases()
    for fitfun in ["Aggregation","Dispersion"]:
        centroids_sdbc = load_centroids("/home/david/argos-sferes/experiments/centroids/centroids_4096_10.dat")
        centroids_spirit = load_centroids("/home/david/argos-sferes/experiments/centroids/centroids_4096_1024.dat")
        for gen in range(5000,5500,500):
            input="/home/david/Data/ExperimentData/"+fitfun+"range11/Gomes_sdbc_walls_and_robots_std/FAULT_NONE/results1/analysis"+str(gen)+"_handcrafted.dat"
            output="/home/david/Data/ExperimentData/"+fitfun+"range11/Gomes_sdbc_walls_and_robots_std/FAULT_NONE/results1/analysis"+str(gen)+"_handcraftedREDUCED.dat"
           #
            # #archive_file, transformation, new_archive_file, transform_data
            reduce_translated_archive(input,mapelites_bd_add,output,transform_data=(16,16,16))
            #
            # input="/home/david/Data/ExperimentData/"+fitfun+"range11/environment_diversity/FAULT_NONE/results1/analysis"+str(gen)+"_spirit.dat"
            # output="/home/david/Data/ExperimentData/"+fitfun+"range11/environment_diversity/FAULT_NONE/results1/analysis"+str(gen)+"_spiritREDUCED.dat"
            # reduce_translated_archive(input,cvt_mapelites_bd_add, output,transform_data=centroids_spirit)

            input="/home/david/Data/ExperimentData/"+fitfun+"range11/Gomes_sdbc_walls_and_robots_std/FAULT_NONE/results1/analysis"+str(gen)+"_sdbc.dat"
            output="/home/david/Data/ExperimentData/"+fitfun+"range11/Gomes_sdbc_walls_and_robots_std/FAULT_NONE/results1/analysis"+str(gen)+"_sdbcREDUCED.dat"
            reduce_translated_archive(input,cvt_mapelites_bd_add, output,transform_data=centroids_sdbc)


# (archive_file,bd_condition,new_archive_file,helper_data)