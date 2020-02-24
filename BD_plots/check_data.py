

"""

utilities to check:
 -which data did not finish yet
 -whcih perturbed archives are incomplete ( compare number of lines to that of the evolved archive)

"""

from process_archive_data import *

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
    sensor_perturbs = 30
    actuator_perturbs = 20
    software_perturbs = 6
    num_food = 5
    max_num_agents = 12

    sensor_perturbations = ["sensorp" + str(i) for i in range(1, sensor_perturbs + 1)]
    actuator_perturbations = ["actuatorp" + str(i) for i in range(1, actuator_perturbs + 1)]
    software_perturbations = ["softwarep" + str(i) for i in range(1, software_perturbs + 1)]
    softwarefood_perturbations = ["software_foodp" + str(i) + "f" + str(f) for i in range(1, software_perturbs + 1) for
                                  f in range(1, num_food + 1)]
    agent_perturbations = ["agentsp" + str(i) for i in range(1, max_num_agents + 1)]
    perturbations = sensor_perturbations + actuator_perturbations + software_perturbations + softwarefood_perturbations + agent_perturbations
    for desc in descriptors:
        print(desc)
        filename = datadir + "/Foraging/" +  desc
        for run in runs:

            unperturbed = filename + "/results" + str(run)+"/archive_"+str(finalgen)+".dat"
            file_length=len(read_spacedelimited(unperturbed))

            U_list = set(get_ind_performances_uniquearchive(unperturbed).keys())

            for perturbation in perturbations:

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
                missing =  U_list - P_list
                if missing:
                    # find missing individuals
                    print(perturbed)
                    print("missing : " + str(missing))
                too_much = P_list - U_list
                if too_much:
                    print(perturbed)
                    print("too much : "+str(too_much))
if __name__ == "__main__":
    # check_archives_complete(30000,
    #                         "/home/david/Data",
    #                         ["Aggregation","Dispersion","DecayCoverage","DecayBorderCoverage","Flocking"],
    #                         ["Gomes_sdbc_walls_and_robots_std", "environment_diversity"],
    #                         range(1,6),
    #                         range(50),
    #                         translation="handcrafted"
    #                         )

    check_archives_complete_foraging(20000,
                            "/home/david/Data",
                            ["history","Gomes_sdbc_walls_and_robots_std"],
                            range(1,6),
                            translation="handcrafted"
                            )