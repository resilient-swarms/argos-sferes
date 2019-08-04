
import numpy as np

def calc_sample_size(sd, desired_width):
    return 16*(sd/desired_width)**2

def calc_cell_dimensions(arena_size,range):
    X,Y = arena_size
    cell_size = 2*range
    return np.ceil(X/cell_size),np.ceil(Y/cell_size)
