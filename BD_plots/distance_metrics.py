
import numpy as np
import time

def Euclidian_dist(p1,p2):

    dist = np.sum(np.square(p1 - p2))

    return dist**0.5

def max_Euclidian_dist(bd_size):
    """
    calculate the Euclidian distance between point (0,0,..,0) and (1,1,..,1)

    we can make it more efficient by noting that sum of squared diffs = bd_size
    :param p1:
    :param p2:
    :return:
    """
    return np.sqrt(bd_size)


def norm_Euclidian_dist(p1,p2):
    return Euclidian_dist(p1,p2)/max_Euclidian_dist(len(p1))



# def relative_entropy(p1,p2):
#     """
#     combine the relative entropy in both directions to ensure symmetric property
#     i.e., the sum D(x||y)+D(y||x)
#     :param p1:
#     :param p2:
#     :return:
#     """
#
#     s = np.sum((p1-p2)*np.log(p1/p2))
#     if np.isnan(s):
#         print("warning: found NAN in relative entropy")
#         time.sleep(2)
#     return s
#
# def avg_relative_entropy(p1,p2,num_states,num_actions):
#
#     for s in range(num_states):
#         np.sum()
#
#     return dist**0.5


def variation_distance(p1,p2):
    return 0.5*np.sum(np.abs(p1-p2))
def avg_variation_distance(p1,p2,num_actions):
    return np.mean([variation_distance(p1[j:j+num_actions],p2[j:j+num_actions]) for j in range(0,len(p1)+num_actions,num_actions)])


