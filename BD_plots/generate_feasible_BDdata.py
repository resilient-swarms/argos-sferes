

# import argparse
#
# parser = argparse.ArgumentParser(description='Process arguments.')
# parser.add_argument('-k', type=int,default=1000,
#                     help='number of data')
import numpy as np
import sys
import pickle
import os
home=os.environ["HOME"]

N = 1000000  # as usual take 100000 datapoints
batch_size = 20000  # avoid too big memory requirement
BD = 1024
centroids = 4096
states = 64
actions = 16
prefix = home + "/argos-sferes/experiments/centroids/SPIRIT"

import sys

def generate_data(start,end,num_states,num_actions,BD_size):
    final_points=[]

    for i in range(start,end):
        #print("point "+str(i))
        point=[]
        for s in range(num_states):
            p=np.zeros(num_actions)
            eps=0.0
            # generate a random index in [0,num_actions-1] to avoid order effects
            indexes = np.random.choice(range(num_actions), num_actions, replace=False)
            for index in indexes[0:-1]:
                # generate a probability in [0,1 - eps], where eps is the probability already allocated
                num = np.random.choice(range(1001))   # generate number between [0,1000]
                num = round(num*(1. - eps)) # create an integer in [0,1000*eps]
                prob = num*.001 # convert to [0,eps]
                p[index]=prob
                eps+=p[index]
                if eps >= 0.999:
                    break
            p[indexes[-1]]=1-eps # assign all that is left to the last element in indexes
            #print(p)
            assert abs(sum(p)-1.0) < 0.0000005
            point = np.append(point,p)
        assert len(point) == BD_size
        #print(point)
        #print(i)
        final_points.append(point)
    print("batch done")
    return final_points
def make_batches(index=0,r=range(0,N,batch_size)):
    assert batch_size>BD
    j=index
    print("will do all the data in range " +str(r[0])+" to "+str(r[-1]))
    for i in r:
        print("starting batch index " + str(j))
        data=generate_data(i,i+batch_size,states,actions,BD)
        pickle.dump(data,open(prefix+str(j),"wb"))
        j+=1
if __name__ == "__main__":
    #import multiprocessing as mp
    #num_cpus=mp.cpu_count()
    #pool = mp.Pool(num_cpus)
    #start=6
    #end=50
    #num_per_proc=int(np.ceil((end-start)/float(num_cpus)))
    #pool.starmap(make_batches, [(i,range(i*batch_size,(i+num_per_proc)*batch_size)) for i in range(start,end,num_per_proc)])
    #pool.close()
    i=int(sys.argv[1])
    print("starting batch")
    make_batches(i,range(i*batch_size,(i+1)*batch_size)) 
    #os.system("python "+home+"/argos-sferes/sferes2/modules/cvt_map_elites/cvt.py -p="+str(N)+" -b="+str(batch_size)+" -d="+str(BD)+" -k="+str(centroids)+" -l="+prefix)
