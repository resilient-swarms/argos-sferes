

"""

utilities to check:
 -which data did not finish yet
 -whcih perturbed archives are incomplete ( compare number of lines to that of the evolved archive)

"""

from process_archive_data import *

def check_archives_complete(finalgen,datadir,fitfuns, descriptors,runs, perturbations):


    for fitfun in fitfuns:
        print(fitfun)
        for desc in descriptors:
            print(desc)
            filename = datadir + "/" + fitfun+"range0.11""/" + desc
            for run in runs:
                unperturbed = filename + "/results" + str(run)+"/archive_"+str(finalgen)+".dat"
                fault_none = filename + "/FAULT_NONE/results" + str(run)+"/analysis"+str(finalgen)+"_handcrafted.dat"
                U_list = set(get_ind_performances_uniquearchive(unperturbed).keys())
                F_list = set(get_ind_performances_uniquearchive(fault_none).keys())

                missing = U_list - F_list
                if missing:
                    # find missing individuals
                    print(perturbed)
                    print("missing : " + str(missing))
                too_much = F_list - U_list
                if too_much:
                    print(perturbed)
                    print("too much : " + str(too_much))

                for perturbation in perturbations:
                    perturbed = filename + "/run"+str(run)+"_p"+str(perturbation)+"/results"+str(run)+"/analysis"+str(finalgen)+"_handcrafted.dat"
                    P_list = set(get_ind_performances_uniquearchive(perturbed).keys())

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
    check_archives_complete(10000,
                            "/home/david/Data/ExperimentData",
                            ["Aggregation","Dispersion","DecayCoverage","DecayBorderCoverage"],
                            ["history","cvt_rab_spirit","Gomes_sdbc_walls_and_robots_std"],
                            range(1,6),
                            range(0,40)
                            )