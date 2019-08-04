

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

N = 100000  # as usual take 100000 datapoints
batch_size = 10000  # avoid too big memory requirement
BD = 6400
centroids = 4096
states = 256
actions = 25
prefix = home + "/argos-sferes/experiments/centroids/SPIRIT"

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
                p[index]=np.random.rand()*(1.-eps)
                eps+=p[index]
                if eps >= 1.00:
                    break
            p[indexes[-1]]=1-eps # assign all that is left to the last element in indexes
            #print(p)
            assert abs(sum(p)-1.0) < 0.0000005
            point = np.append(point,p)
        assert len(point) == BD_size
        #print(point)
        final_points.append(point)
    return final_points
def make_batches():
    assert batch_size>BD
    j=0
    for i in range(0,N,batch_size):
        data=generate_data(i,i+batch_size,states,actions,BD)
        pickle.dump(data,open(prefix+str(j),"wb"))
        j+=1
if __name__ == "__main__":
    make_batches()
    os.system("python "+home+"/argos-sferes/sferes2/modules/cvt_map_elites/cvt.py -p="+str(N)+" -b="+str(batch_size)+" -d="+str(BD)+" -k="+str(centroids)+" -l="+prefix)