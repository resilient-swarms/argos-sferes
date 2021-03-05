import numpy as np

import matplotlib.pyplot as PLT

from process_archive_data import *
ro=0.10
KAPPA=0.80

def euclid(x,y):
    return np.sqrt(np.sum(np.square(x-y)))

def k(x,y,dist=euclid):
     global ro
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

def sigma(x,samp,K_inv):
    if not samp:
        return k(x,x)
    print("k(x,x)"+str(k(x,x)))
    print("kKk" + str(small_k(x,samp).dot(K_inv).dot(small_k(x,samp,True))))
    return k(x,x) - small_k(x,samp).dot(K_inv).dot(small_k(x,samp,True))

def mu(prior,x,samples,observations,corresponding_priors,K_inv):
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
def phi(x,xj):
    print("WARNING: local penalisation phi not implemented yet")
    dist=np.sum(np.square(x - xj))
    return  1.0 if dist > 0.1 else dist*2.

def get_max_acquisition(bds,priors,ys,xs,queried_priors,K_inv,excluded_indexes=[],get_all=False,busy_samples=[]):
    # compute the mean for all the points in the archive

    indexes=range(len(bds))
    max_ind = None
    max_acq = -float("inf")
    evaluated_mu=[]
    evaluated_sd=[]
    for i in range(len(indexes)):
        x = bds[i]
        M = mu(priors[i], x, xs, np.array(ys), np.array(queried_priors), K_inv).flatten()[0]
        print("mu =" + str(M))
        s = sigma(x, xs, K_inv).flatten()[0]
        print("update effect = ", M - priors[i])  # inverse relation between update effect size and noise
        if i not in excluded_indexes:
            UCB = M + KAPPA*s
            if busy_samples:
                UCB = UCB * np.prod([phi(x,bds[bs])  for bs in busy_samples])
            if UCB > max_acq:
                max_acq =UCB
                max_ind = i

        if get_all:
            evaluated_mu.append(M)
            evaluated_sd.append(s)

    print("checked the archive")
    print("max acq = " + str(max_acq))
    # select index in the remaining list
    if get_all:return max_ind, np.array(evaluated_mu), np.array(evaluated_sd)
    return max_ind

if __name__ == "__main__":
    noise=0.01
    samples=[np.array([0,0.50,1.0]),np.array([1.0,0.45,1.0]),np.array([0.0,0.50,0.0])]
    observations=[1.0,0.5,0.2]
    Kn = K(samples) + noise * np.identity(3)
    K_inv = np.linalg.inv(Kn)


    print(k(samples[0],samples[0]))
    print("sigma "  + str(sigma(samples[0],samples,K_inv)))
    print("sigma "  + str(sigma(samples[1],samples,K_inv)))
    print("sigma "  + str(sigma(samples[2],samples,K_inv)))
    print("sigma "  + str(sigma(np.array([0.1,0.1,0.1]),samples,K_inv)))


    x_values=np.linspace(0,7,100)
    reference=0
    for r in [0.1,0.2,0.5,1.0,2.0]:
        ro=r
        y_values=[k(0,x) for x in x_values]
        PLT.plot(x_values,y_values)
    PLT.savefig("kernel.pdf")


    print("checking prior\n----\n")

    bds,priors=load_archive()
    excluded_indexes=[]
    samples=[]
    observations=[]
    queried_priors=[]
    noises=[]
    K_inv=[]
    while len(excluded_indexes) < len(bds):
        j = get_max_acquisition(bds,priors,observations,samples,queried_priors,K_inv,excluded_indexes,get_all=False)
        x = bds[j]
        excluded_indexes.append(j) # delete that item
        # now add to the observations
        prior=priors[j]

        samples.append(x)
        observations.append(np.random.randint(0,3)) # low values (0-2) give negative means
        queried_priors.append(prior)
        noises.append(0.0) # only extreme noise seems to avoid negative values
        #noises.append(observations[-1] * observations[-1]) # large/variable noise is not cause of negative values
        #update the kernel matrix
        noise_mat = np.array(noises)*np.identity(len(samples))
        Kn = K(samples)
        Kn = Kn + noise_mat
        K_inv = np.linalg.inv(Kn)



