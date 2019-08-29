
import numpy as np


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
