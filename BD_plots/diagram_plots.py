


from plots import *
from kernel_checks import *
from copy import deepcopy
global K_inv

def BP_prior(B):
    factors = [12.5 * min(b, 0.80) for b in B]
    P = 1 * np.cos(50 * B) + 2.5 * np.cos(8 * B - 4) + 1.25 * np.sin(8 * B - 4) + 2.5 + factors
    return P
def behaviour_performance_map():
    B = np.linspace(0,0.99,100)
    P = BP_prior(B)
    createPlot([P],B,colors=["grey"],markers=".",xlabel=r"Behaviour ($\mathbf{x}$)",ylabel=r"Performance ($f(\mathcal{M}[\mathbf{x}]$)",
               ylim=None,save_filename="diagram_behaviour_performance_map.pdf",legend_labels=["nothing"],skip_legend=True,force=True,
               scatter=True,title="Behaviour-performance map")
    print("now do the prior")
    # \mu(x) set to the mean
    # \sigma(x) set to the k(x,x) + noise variance
    sigma=k(B,B)
    bottom=[P - sigma]
    top   =[P + sigma]
    createPlot([P],B,colors=["grey"],markers=".",xlabel=r"Behaviour ($\mathbf{x}$)",
               ylabel=r"Performance prior ($f(\mathbf{x})$)",
               ylim=None,save_filename="diagram_behaviour_prior.pdf",legend_labels=["nothing"],skip_legend=True,force=True,
               scatter=False,title="Bayesian prior",fill_between=[bottom,top])
    return B,P

def approximate(P,excluded_indexes,bds, samples,observations,queried_priors,K_inv,busy_samples=[]):
    j, mus,sds = get_max_acquisition(bds=bds,priors=P,ys=observations,xs=samples,
                                     queried_priors=queried_priors,K_inv=K_inv,
                                     excluded_indexes=excluded_indexes,get_all=True,busy_samples=busy_samples)
    return j, mus, KAPPA*sds

def application(B,P):
    # start with prior --> get maximal

    excluded_indexes = []
    # update F
    i, F, sds = approximate(P, excluded_indexes, B, samples=[],observations=[],queried_priors=[],K_inv=[])
    excluded_indexes.append(i)
    annot= {"text": r"$\mathbf{x}_1 = argmax_{\mathbf{x} \in \mathcal{M}} UCB(\mathbf{x})$",
           "xy":(B[i],F[i]+sds[i]), "xytext":(B[i]-0.3,F[i]+sds[i]-0.2),"align":"center","fontsize":25}
    createPlot([F],B,colors=["grey"],markers=".",xlabel=r"Behaviour ($\mathbf{x}$)",
               ylabel=r"Performance prior ($f(\mathbf{x})$)",
               ylim=None,save_filename="diagram_select_prior.pdf",legend_labels=["nothing"],skip_legend=True,force=True,
               scatter=False,title=r"Prior ",fill_between=[[F-sds],[F+sds]],annotations=[annot])

    # now add the sample and update
    samples = [B[i]]
    observations = [5.5]
    prior = P[i]
    queried_priors=[prior]
    noises=[0.01]
    noise_mat = np.array(noises) * np.identity(len(samples))
    Kn = K(samples)
    Kn = Kn + noise_mat
    K_inv = np.linalg.inv(Kn)


    # update F
    j, F, sds = approximate(P, excluded_indexes, B, samples=samples,
                            observations=observations,queried_priors=queried_priors,K_inv=K_inv)
    excluded_indexes.append(j)
    annot= {"text": r"$\mathbf{x}_2 = argmax_{\mathbf{x} \in \mathcal{M}} UCB(\mathbf{x})$",
           "xy":(B[j],F[j]+sds[j]), "xytext":(B[j]-0.3,F[j]+sds[j]-0.2),"align":"center","fontsize":25}
    annot2={"text": r"update $f(\mathbf{x}_1)$",
           "xy":(B[i],F[i]+sds[i]), "xytext":(B[i],F[i]+sds[i]-1.8),"align":"center","fontsize":25}
    createPlot([F],B,colors=["grey"],markers=".",xlabel=r"Behaviour ($\mathbf{x}$)",
               ylabel=r"Performance posterior ($f(\mathbf{x})$)",
               ylim=None,save_filename="diagram_SMBO.pdf",legend_labels=["nothing"],skip_legend=True,force=True,
               scatter=False,title=r"SMBO, $\mathcal{D}=(\mathbf{x}_1,f(\mathbf{x}_1))$",
               fill_between=[[F-sds],[F+sds]],annotations=[annot,annot2])
    print()

    # SMBO-Dec
    l, F, sds = approximate(P, excluded_indexes, B, samples=samples, observations=observations,
                            queried_priors=queried_priors, K_inv=K_inv,busy_samples=[j])
    excluded_indexes.append(l)
    annot3= {"text": r"$\mathbf{x}_3 = argmax_{\mathbf{x} \in \mathcal{M}} UCB(\mathbf{x}) \phi(\mathbf{x},\mathbf{x}_2)$",
           "xy":(B[l],F[l]+sds[l]), "xytext":(B[l]-0.16,F[l]+sds[l]+0.01),"align":"center","fontsize":25}
    fill_between=[[F-sds],[F+sds]]
    print(F-sds)
    print(F + sds)
    createPlot([F],B,colors=["grey"],markers=".",xlabel=r"Behaviour ($\mathbf{x}$)",
               ylabel=r"Performance posterior ($f(\mathbf{x})$)",
               ylim=None,save_filename="diagram_SMBO-Dec.pdf",legend_labels=["nothing"],skip_legend=True,force=True,
               scatter=False,title=r"SMBO-Dec, $\mathcal{D}=(\mathbf{x}_1,f(\mathbf{x}_1))$ and $\mathcal{B}=\{\mathbf{x}_2\}$",
               fill_between=fill_between,
               annotations=[annot,annot2,annot3])
    print()






    # another new sample
    Kn = K(samples) + noise * np.identity(1)
    K_inv = np.linalg.inv(Kn)
    max_ind = get_max_acquisition(priors=P,ys=observations,xs=samples,queried_priors=[],K_inv=K_inv,remaining_indexes=remaining_indexes,get_all=True)
    createPlot([P],B,colors=["grey"],markers=".",xlabel=r"Behaviour ($\mathbf{x}$)",
               ylabel=r"Performance prior ($f(\mathbf{x})$)",
               ylim=None,save_filename="behaviour_prior.pdf",legend_labels=["nothing"],skip_legend=True,force=True,
               scatter=False,title="Bayesian prior",fill_between=[bottom,top])



if __name__ =="__main__":
    B,P = behaviour_performance_map()
    application(B,P)