import numpy as np

import matplotlib.pyplot as PLT
ro=0.4

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


noise=0.001
samples=[np.array([0,0.50,1.0]),np.array([1.0,0.45,1.0]),np.array([0.0,0.50,0.0])]
observations=[1.0,0.5,0.2]
Kn=K(samples) + noise*np.identity(3)
K_inv = np.linalg.inv(Kn)


print(k(samples[0],samples[0]))
print(sigma(samples[0],samples))
print(sigma(samples[1],samples))
print(sigma(samples[2],samples))
print(sigma(np.array([0.1,0.1,0.1]),samples))


x_values=np.linspace(0,1,10)
reference=0
y_values=[k(0,x) for x in x_values]
PLT.plot(x_values,y_values)
PLT.savefig("kernel.pdf")
