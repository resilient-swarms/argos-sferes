import argparse
import sys, os
from process_archive_data import *
HOME_DIR = os.environ["HOME"]



"""
    
"""
# def replace(template_file):
#     return "sed -e "
def make_and_get_configs(replicate):
    configs=[]
    ConfigFile=args.p+"/exp"+str(replicate)
    template_file=HOME_DIR+"/argos-sferes/experiments/harvesting/environment_template_harvesting.argos"
    maxspeeds=[5,10,15]
    robots=[3,6,12]
    walls=[4]
    cylinders=[2]
    proxiranges=[0.055,0.11,0.22]
    groundnoises=[10,20,40]
    for c1, MaxSpeed in enumerate(maxspeeds):
        for c2, Robots in enumerate(robots):
            for c3, Wall in enumerate(walls):
                WallThickness = 1.0
                HalfWall = Wall / 2  #
                Arena = Wall+2 * WallThickness  # wall + 2m to account for 2*1m wall
                Center = HalfWall
                WallOff = Wall-0.5   # just for init robots

                FullWall = Wall + WallThickness  / 2.0
                for c4, Cylinder in enumerate(cylinders):
                    for c5, GroundNoise in enumerate(groundnoises):
                        for c6, ProxiRange in enumerate(proxiranges):
                            ConfigTag = str(c1+1) + "," + str(c2+1) + "," + str(c3+1) + "," + str(c4+1) + "," + str(c5+1) + "," + str(c6+1)
                            cfg = ConfigFile +"_"+ConfigTag+".argos"
                            command="sed -e " +\
                            "'s|THREADS|0|' " + \
                            " -e "+ \
                            "'s|TRIALS|8|'" +\
                            " -e " +\
                            "'s|ROBOTS|"+str(Robots)+"|'" +\
                            " -e " +\
                            "'s|EXPERIMENT_LENGTH|120|'" +\
                            " -e " +\
                            "'s|SEED|"+str(replicate)+"|'" +\
                            " -e " +\
                            "'s|FITFUN_TYPE|Foraging|'" +\
                            " -e " +\
                            "'s|DESCRIPTOR_TYPE|analysis|'" +\
                            " -e " +\
                            "'s|OUTPUTFOLDER|"+args.o+"/results"+str(replicate)+"|'" +\
                            " -e " +\
                            "'s|SENSOR_RANGE|"+str(ProxiRange)+"|'" +\
                            " -e " +\
                            "'s|CENTROIDSFOLDER|experiments/centroids|' " +\
                            " -e " +\
                            "'s|NOISE_LEVEL|0.05|'" +\
                            " -e " +\
                            "'s|BEHAVIOUR_TAG|3DANAREAL|'" +\
                            " -e " +\
                            "'s|MAX_SPEED|"+ str(MaxSpeed) + "|'" +\
                            " -e " +\
                            "'s|ARENA,ARENA|"+str(Arena)+","+str(Arena)+"|'" +\
                            " -e " +\
                            "'s|ARENA|"+str(Arena)+"|'" +\
                            " -e " +\
                            "'s|CENTER,CENTER|"+str(Center)+","+str(Center)+"|'" +\
                            " -e " +\
                            "'s|FULL_WALL|"+str(FullWall)+"|'" +\
                            " -e " +\
                            "'s|HALF_WALL|"+str(FullWall)+"|'"  +\
                            " -e " +\
                            "'s|WALL_OFF,WALL_OFF|"+str(WallOff)+","+str(WallOff)+"|'" +\
                            " -e " +\
                            "'s|NUM_CYLINDERS|"+str(Cylinder)+"|'" +\
                            " -e " +\
                            "'s|GROUND_NOISE|"+str(GroundNoise)+"|' " +\
                            template_file + " > "+ cfg
                            print("will write to ",cfg)
                            configs.append((cfg,Robots))
                            os.system(command)
    return configs

# template_file +

def discretise(vec,n_bins):
    bin_size=1/float(n_bins)
    for d in range(len(vec)):
        vec[d] = (vec[d]//bin_size)*bin_size
    return vec

def form_bd_string(bd):
    s = ""
    for i in range(len(bd)):
        s+=str(bd[i])
        s+=" "
    return s

def run_id_map(replicate):

    #initialise
    path = args.p + "/archive_"+args.g+".dat"
    parsed_file_list = read_spacedelimited(path)
    individuals = []
    for item in parsed_file_list:
        i = str(item[0])
        BD = list(np.array(item[1:-1],dtype=float))
        individuals.append((i,BD))
    configs = make_and_get_configs(replicate)
    bp_map=OrderedDict({})
    n_bins=16

    # form the map
    analysis_file=args.o+"/results"+str(replicate)+"/analysis"+str(args.g)+"_identification.dat"
    os.system("rm "+analysis_file)
    for (i,BD) in individuals:
        for config,num_robots in configs:
            command = "cd "+HOME_DIR+"/argos-sferes && "+args.c+ " " +config+ " identification "+args.g+" -d " +args.o+"/results"+str(replicate)+" --load " +args.p+ "/gen_" +args.g+ " -o outputfile "
            run_individual(command, i)
            parsed_file_list = read_spacedelimited(analysis_file)
            j=0
            for line in parsed_file_list:
                ID = discretise(list(np.array(line[1:-1],dtype=float)),n_bins)
                b = form_bd_string(BD+ID)
                f = float(line[-1])
                j+=1
                if (j > num_robots):
                    raise Exception("line number greater than robots")
                N, f_old, S_old = bp_map.get((i,b),(0,0.,0.))
                N+=1
                M = ((N-1)*f_old + f)/float(N)
                S = S_old + (f - f_old) * (f - M)
                bp_map[(i,b)] = (N, M, S )

            os.system("rm " + analysis_file)

    #write the map to readable format for the c++ code
    ID_archive_filename = args.o + "/results" + str(replicate)+"/ID_archive_"+str(args.g)+".dat"
    ID_archive_file = open(ID_archive_filename,"w")
    for key, val in bp_map.items():
        i,b = key
        _N, f,S = val
        sd = S/float(_N - 1) if _N > 1 else f**2
        ID_archive_file.write(i + " "+str(b) + " " + str(f) + " " + str(sd) + "\n")
    print("finished archive ", ID_archive_filename)

if __name__ == "__main__":
    run_id_map(args.r)