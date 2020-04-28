
import numpy as np

def good_solution_density(min_reference,max_reference, performances):
    count = 0.0
    for p in performances:
        if p > min_reference + 0.50*(max_reference - min_reference):
            count+=1.0
    return count/float(len(performances))

def von_neumann_neighbourhood(bd, step_size=0.0625):
    D=len(bd)
    return list(bd+step_size*np.identity(D)) + list(bd-step_size*np.identity(D)) # perturb each dimension by one step
def smoothness(parsed_list,neighbourhood_function, step_size=0.0625):
    d={ str(tuple(line[0:-1])) : float(line[-1]) for line in parsed_list}
    grand_avg = np.mean([float(line[-1]) for line in parsed_list])
    sqdev = 0.0
    sqdev_neighbours = 0.0
    for line in parsed_list:
        bd = np.array(line[0:-1],dtype=float)
        fitness = float(line[-1])
        # help compute sd
        sqdev += (fitness - grand_avg)**2

        # check if neighbours exist in the map
        #print(b_array)
        neighbours = neighbourhood_function(bd, step_size)
        neighbours = [ str(tuple(n)) for n in neighbours] # convert to string
        neighbours = [n for n in neighbours if n in d] # check if in dict
        if neighbours:
            sqdev_neighbours += np.sum([(fitness - d[n])**2 for n in neighbours])/len(neighbours) # filter out the ones that don't exist

    return sqdev/sqdev_neighbours




if __name__ == "__main__":
    X,Y = np.mgrid[0:1:0.1, 0:1:0.1]
    x = np.vstack((X.flatten(), Y.flatten())).T
    y = np.sum(x,axis=1)

    print()
    bd_fitness_list=[]
    for i in range(x.shape[0]):
            bd_fitness_list.append(list(x[i]) + [y[i]])

    smooth = smoothness(bd_fitness_list,von_neumann_neighbourhood, step_size=0.1)
    print("smoothness = " +str(smooth))


    print()
    bd_fitness_list=[]
    for i in range(x.shape[0]):
            bd_fitness_list.append(list(x[i]) + [np.random.random()])
    smooth = smoothness(bd_fitness_list, von_neumann_neighbourhood, step_size=0.1)
    print("smoothness = " + str(smooth))


    dens = good_solution_density(min_reference=10,max_reference=15,performances=[13,13,13])

    print("density ="+str(dens))

    dens = good_solution_density(min_reference=10,max_reference=15,performances=[11,13,13])

    print("density ="+str(dens))