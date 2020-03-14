argos-sferes
=======

An interface for using the ARGoS swarm robotics simulator with the Sferes evolutionary algorithm framework.

The argos-sferes repository has been used in the paper:

Bossens, D. M., & Tarapore, D. (2020). QED: using Quality-Environment-Diversity to evolve resilient robot swarms. ArXiv:2003.02341 Preprint. Retrieved from http://arxiv.org/abs/2003.02341


Instruction for compilation (using MAP-Elites and neural-network modules)
-------------

1. Install ARGoS https://www.argos-sim.info/ (via binary packages or source)

   Install Thymio simulation plugin for ARGoS from https://github.com/resilient-swarms/thymio

2. Download argos-sferes code

       git clone https://github.com/daneshtarapore/argos-sferes.git


3. Now download the sferes2 code -- all header files, so don't need compilation (unless you want to run the sferes2 test cases).

        cd argos-sferes

        git clone https://github.com/sferes2/sferes2.git 


4. 
A. As nn2, map_elites and cvt_map_elites code are in header files, they don't need to be compiled -- so no need to add them to modules.conf

    cd sferes2/modules

    git clone https://github.com/resilient-swarms/map_elites.git  --branch generic_print

    git clone https://github.com/sferes2/nn2.git
    
    git clone https://github.com/resilient-swarms/cvt_map_elites.git 
   
B. clone limbo for bayesian optimisation (optional):

    cd ../..
    
    git clone https://github.com/resibots/limbo.git
    


5. Compilation:

(a) To compile serial evolution experiments:

   
    bash cmake_scripts/make_all_simulation.sh
    

(b) To compile parallel evolution experiments:

    
    bash cmake_scripts/make_all_parallel.sh
    

(c) To compile evolution experiments with parallel environments:


    bash cmake_scripts/make_all_envirparallel.sh
    

(d) To compile Bayesian optimisation:


    bash cmake_scripts/make_all_BO.sh
    
    

(e) To compile baseline behaviours:


    bash make_baseline.sh
    
    


Instruction for running
-------------




Evolution in the normal operating environment can be run using:

   
     ./bin/behaviour_evolcvt10D experiments/Gomes_walls_and_robots_std.argos
   
 
Note here the binaries are organised as "behaviour_evol"${CVT}${DIM}D" where CVT is whether or not using CVT-MAPElites or MAPElites and DIM is the dimensionality of the behavioural descriptor.


Hand-crafted behaviours can also be experimented with just to see how well evolution compares:



     ./bin/baseline_behaviour experiments/baseline-behavs.argos
   


Running the experiments from the above-mentioned paper can be done using the "launch" scripts:


For QED, this is:

      bash launch_environment_descriptor.sh ${data_directory}
   
   
For the baseline QD-algorithms, this is:


      bash launch_Gomes_jobs.sh ${data_directory}


To try out an evolved solution

     ./bin/behaviour_evol2D experiments/history.argos --load <path to generation file>/gen_<number> -o <output file> -n <index of individual in MAP>

To try out Bayesian Optimisation do:

     bin/ite_swarms_3D -m <data_directory> 20000 -e bin/BO3D 
     experiments/history_BO.argos
