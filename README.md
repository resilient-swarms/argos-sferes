argos-sferes
=======

An interface for using the ARGoS swarm robotics simulator with the Sferes evolutionary algorithm framework.


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
   
B. clone limbo for bayesian optimisation:

    cd ~/argos-sferes
    
    git clone https://github.com/resibots/limbo.git
    

5. Compilation:
(a) To compile serial evolution experiments:
   
    bash make_all_simulation.sh

(b) To compile parallel evolution experiments:
    
    bash make_all_parallel.sh

(c) To compile evolution experiments with parallel environments:

    bash make_all_envirparallel.sh

(d) To compile Bayesian optimisation:

    bash make_all_BO.sh

(e) To compile baseline behaviours:

    bash make_baseline.sh


Instruction for running
-------------




Multi-agent experiments can be run using 
   
   ./bin/behaviour_evolcvt10D experiments/Gomes_walls_and_robots_std.argos

    
    ./bin/behaviour_evolcvt576D experiments/multiagent_spirit.argos

and:

   ./bin/baseline_behaviour experiments/baseline-behavs.argos

Multiple environments can be run with 

    ./bin/envir_evolcvt10D experiments/generator experiments/redirect

Note that the argument will be automatically supplied with a number and a .argos tag during the run.



For single-agent experiments:
To run the EA, on the setting mentioned in point 5., you can run

    cd argos-sferes
     ./bin/behaviour_evol2D experiments/history.argos

To run the EA on other settings mentioned in point 6., you can run either of the following commands:

     ./bin/behaviour_evolcvt14D experiments/mutualinfoact.argos
     ./bin/behaviour_evolcvt21D experiments/mutualinfo.argos
     ./bin/behaviour_evolcvt400D experiments/spirit.argos

To try out an evolved solution

     ./bin/behaviour_evol2D experiments/history.argos --load <path to generation file>/gen_<number> -o <output file> -n <index of individual in MAP>

To try out Bayesian Optimisation do:

     bin/ite_swarms_3D -m <data_directory> <generation> -e bin/BO3D experiments/realrobot_testBO.argos
