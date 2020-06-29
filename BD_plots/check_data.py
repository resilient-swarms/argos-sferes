

"""

utilities to check:
 -which data did not finish yet
 -whcih perturbed archives are incomplete ( compare number of lines to that of the evolved archive)

"""

from os import path
from process_archive_data import *

from foraging_params import *

def check_archives_complete(finalgen,datadir,fitfuns, descriptors,runs, perturbations,translation):


    for fitfun in fitfuns:
        print(fitfun)
        for desc in descriptors:

            print(desc)
            filename = datadir + "/" + fitfun+"range0.11""/" + desc



            for run in runs:

                if desc == "baseline":
                    if len(read_spacedelimited(filename + "/FAULT_NONE/results" + str(run) + "/fitness")) != 1:
                        print(filename)
                    continue
                unperturbed = filename + "/results" + str(run)+"/archive_"+str(finalgen)+".dat"
                fault_none = filename + "/FAULT_NONE/results" + str(run)+"/analysis"+str(finalgen)+"_"+translation+".dat"
                file_length=len(read_spacedelimited(unperturbed))
                file_length2=len(read_spacedelimited(fault_none))

                if file_length!=file_length2:
                    print(fault_none)
                    print("file length not equal")

                U_list = set(get_ind_performances_uniquearchive(unperturbed).keys())
                F_list = set(get_ind_performances_uniquearchive(fault_none).keys())

                missing = U_list - F_list
                if missing:
                    # find missing individuals
                    print(fault_none)
                    print("missing : " + str(missing))
                too_much = F_list - U_list
                if too_much:
                    print(fault_none)
                    print("too much : " + str(too_much))

                for perturbation in perturbations:
                    if desc == "baseline":
                        if len(read_spacedelimited(filename + "/faultyrun"+str(run)+"_p"+str(perturbation)+"/results"+str(run)+"/fitness")) != 1:
                            print(filename)
                        continue

                    perturbed = filename + "/faultyrun"+str(run)+"_p"+str(perturbation)+"/results"+str(run)+"/analysis"+str(finalgen)+"_"+translation+".dat"
                    try:
                        P_list = set(get_ind_performances_uniquearchive(perturbed).keys())
                        file_length3 = len(read_spacedelimited(perturbed))
                    except Exception as e:
                        print(e)
                        P_list=set([])
                        file_length3=0
                    if file_length!=file_length3:
                        print(perturbed)
                    missing =  U_list - P_list
                    if missing:
                        # find missing individuals
                        print(perturbed)
                        print("missing : " + str(missing))
                    too_much = P_list - U_list
                    if too_much:
                        print(perturbed)
                        print("too much : "+str(too_much))

def check_archives_complete_foraging(finalgen,datadir,descriptors,runs, translation):

    for desc in descriptors:
        print(desc)
        filename = datadir + "/Foraging/" +  desc
        for run in runs:

            unperturbed = filename + "/results" + str(run)+"/archive_"+str(finalgen)+".dat"
            file_length=len(read_spacedelimited(unperturbed))

            U_list = set(get_ind_performances_uniquearchive(unperturbed).keys())

            for perturbation in foraging_perturbations:

                perturbed = filename + "/faultyrun"+str(run)+"_"+perturbation+"/results"+str(run)+"/analysis"+str(finalgen)+"_"+translation+".dat"
                try:
                    P_list = set(get_ind_performances_uniquearchive(perturbed).keys())
                    file_length3 = len(read_spacedelimited(perturbed))
                except Exception as e:
                    print(e)
                    P_list=set([])
                    file_length3=0
                if file_length!=file_length3:
                    print(perturbed)
                    print("file length not the same")

                missing =  U_list - P_list
                if missing:
                    # find missing individuals
                    print("missing : " + str(missing))
                too_much = P_list - U_list
                if too_much:
                    print("too much : "+str(too_much))
                if not missing and not too_much and file_length!=file_length3:
                    print("must have duplicates")
def check_BO_complete_foraging(datadir,descriptors,runs):


    for desc in descriptors:
        print(desc)
        filename = datadir + "/Foraging/" +  desc
        for run in runs:

            for perturbation in foraging_perturbations:

                perturbed = filename + "/faultyrun"+str(run)+"_"+perturbation+"/results"+str(run)+"/BO_output"
                if not path.exists(perturbed):
                    print("could not find:")
                    print(perturbed)

def check_BO_single_complete_foraging(datadir,descriptors,runs,methods,tag):


    for desc in descriptors:
        print(desc)
        filename = datadir + "/Foraging/" +  desc
        for method in methods:
            print("--------------")
            print(method)
            print("--------------")
            for run in runs:
                print("run ",run)
                print("-------")
                for perturbation in ["proximity_sensorp1"]:

                    perturbed = filename + "/faultyrun"+str(run)+"_"+perturbation+"/results"+str(run)+"/"+method+"/BO_output"+tag+"/fitness"
                    if not path.exists(perturbed):
                        print("could not find:")
                        print(perturbed)
                    lines = read_spacedelimited(perturbed)
                    if not lines:
                        print("no line(s) found ", perturbed)

                    for i in range(6):
                        perturbed = filename + "/faultyrun"+str(run)+"_"+perturbation+"/results"+str(run)+"/"+method+"/BO_output"+tag+"/async_stats"+str(i)+".dat"
                        if not path.exists(perturbed):
                            print("could not find:")
                            print(perturbed)
                        lines = read_spacedelimited(perturbed)
                        if not lines:
                            print("no line(s) found ", perturbed)
                        first_line = lines[0]
                        print("worker ", i)
                        print("bd 1st controller: ", first_line[1:4])
                        print("performance 1st controller: ", first_line[-1])
                        print("-------")

if __name__ == "__main__":
    # check_archives_complete(30000,
    #                         "/home/david/Data",
    #                         ["Aggregation","Dispersion","DecayCoverage","DecayBorderCoverage","Flocking"],
    #                         ["Gomes_sdbc_walls_and_robots_std", "environment_diversity"],
    #                         range(1,6),
    #                         range(50),
    #                         translation="handcrafted"
    #                         )

    # check_archives_complete_foraging(20000,
    #                         "/home/david/Data",
    #                         ["history"],
    #                         range(1,6),
    #                         translation="handcrafted"
    #                         )

    check_BO_single_complete_foraging( "/home/david/Data",
                            ["history"],
                            range(1,6),
                            ["single_exp_known","single_exp","single_exp_random"],
                            "reset_nocollisionstop")

