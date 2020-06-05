import numpy as np

import matplotlib.pyplot as PLT

from process_archive_data import *
ro=0.121697



def euclid(x,y):
    return np.sqrt(np.sum(np.square(x-y)))

def k(x,y,dist=euclid):
     return (1. + (np.sqrt(5.)*dist(x,y)/ro + 5.*dist(x,y)**2 /(3.*ro**2) ))*(np.exp(-np.sqrt(5.)/ro*dist(x,y)))

def K(x):
     mat=np.zeros(shape=(len(x),len(x)),dtype=float)
     for i in range(len(x)):
       for j in range(len(x)):
           mat[i,j]=k(x[i],x[j])
     return mat

def small_k(x,samp,transpose=False):
    mat = np.array([[k(x,s)  for s in samp]]) #1 x N
    if transpose:
        return mat.T
    else:
        return mat

def sigma(x,samp):
    print("k(x,x)"+str(k(x,x)))
    print("kKk" + str(small_k(x,samp).dot(K_inv).dot(small_k(x,samp,True))))
    return k(x,x) - small_k(x,samp).dot(K_inv).dot(small_k(x,samp,True))

def mu(prior,x,samples,observations,corresponding_priors):
    #cf http://www.resibots.eu/limbo/guides/limbo_concepts.html#state-based-bo
    # compute difference between observations and priors
    if len(observations)==0:
        return prior
    diff = observations - corresponding_priors
    # add difference component to the prior performance for the queried point x
    kK = small_k(x,samples).dot(K_inv)
    return prior + kK.dot(diff)

def load_archive():
    parsed_file_list = read_spacedelimited("/home/david/Data/Foraging/history/results1/archive_20000.dat")
    bds=[]
    priors=[]
    for item in parsed_file_list:
        b=np.array(tuple(item[1:-1]),dtype=float)
        performance=float(item[-1])
        bds.append(b)
        priors.append(performance)
    return bds,priors

noise=0.01
samples=[np.array([0,0.50,1.0]),np.array([1.0,0.45,1.0]),np.array([0.0,0.50,0.0])]
observations=[1.0,0.5,0.2]
Kn = K(samples) + noise * np.identity(3)
K_inv = np.linalg.inv(Kn)


print(k(samples[0],samples[0]))
print("sigma "  + str(sigma(samples[0],samples)))
print("sigma "  + str(sigma(samples[1],samples)))
print("sigma "  + str(sigma(samples[2],samples)))
print("sigma "  + str(sigma(np.array([0.1,0.1,0.1]),samples)))


x_values=np.linspace(0,1,100)
reference=0
y_values=[k(0,x) for x in x_values]
PLT.plot(x_values,y_values)
PLT.savefig("kernel.pdf")


print("checking prior\n----\n")

bds,priors=load_archive()
remaining_indexes=list(range(len(bds)))
samples=[]
observations=[]
queried_priors=[]
noises=[]
while remaining_indexes:

    print("remaining: ", len(remaining_indexes))

    # compute the mean for all the points in the archive
    max_ind=None
    max_acq=-float("inf")
    for i in range(len(remaining_indexes)):
        x = bds[i]
        M = mu(priors[i],x,samples,np.array(observations),np.array(queried_priors))
        print("mu =" + str(M))
        print("update effect = " , M-priors[i]) # inverse relation between update effect size and noise
        if M > max_acq:
            max_acq=M
            max_ind=i

    print("checked the archive")
    print("max acq = " + str(max_acq))
    # select index in the remaining list
    j=remaining_indexes[max_ind] # get the corresponding index
    x=bds[j]
    del remaining_indexes[max_ind] # delete that item
    # now add to the observations
    prior=priors[j]

    samples.append(x)
    observations.append(np.random.randint(0,3)) # low values (0-2) give negative means
    queried_priors.append(prior)
    noises.append(400) # only extreme noise seems to avoid negative values
    #noises.append(observations[-1] * observations[-1]) # large/variable noise is not cause of negative values
    #update the kernel matrix
    noise_mat = np.array(noises)*np.identity(len(samples))
    Kn = K(samples)
    Kn = Kn + noise_mat
    K_inv = np.linalg.inv(Kn)



