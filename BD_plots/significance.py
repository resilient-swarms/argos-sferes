
from scipy.stats import *
import numpy as np

def cliffs_delta(U,x,y):
    """

    meaning: proportion x>y minus proportion y>x
    |d|<0.147 "negligible", |d|<0.33 "small", |d|<0.474 "medium", otherwise "large"

    here calculate based on relation with the rank-sum test
    :param U: the result of the Wilcoxon rank-test/Mann-Withney U-test
    :return:
    """
    m=len(x)
    n=len(y)

    # delta =  2.0*U/float(m*n) - 1.0
    if len(x) > 2000 or len(y)>2000:   # avoid memory issues
        print("starting with lengths %d %d "%(m,n))
        print("digitising samples")
        xspace=np.linspace(x.min(),x.max(),500)
        yspace = np.linspace(x.min(), x.max(), 500)
        freq_x=np.histogram(x, bins=xspace)[0]
        freq_y=np.histogram(y, bins=yspace)[0]

        count=0
        for i in range(len(freq_x)):
            for j in range(len(freq_y)):
                num_combos=freq_x[i]*freq_y[j]
                xx=xspace[i]
                yy=yspace[j]
                if xx > yy:
                    count+=num_combos
                else:
                    count-=num_combos
        count/=float(m*n)
    else:
        z=np.array([xx - yy for xx in x for yy in y]) # consider all pairs of data
        count=float(sum(z>0) - sum(z<0))/float(m*n)
    # assert count==delta, "delta:%.3f  count:%.3f"%(delta,count)
    assert np.sign(U) == np.sign(count)
    label = None
    magn=abs(count)
    if magn < 0.11:
        label="negligible"
    elif magn < 0.28:
        label="small"
    elif magn < 0.43:
        label="medium"
    else:
        label="large"
    return count, label