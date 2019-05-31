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


4. As nn2, map_elites and cvt_map_elites code are in header files, they don't need to be compiled -- so no need to add them to modules.conf

    cd sferes2/modules

    git clone https://github.com/resilient-swarms/map_elites.git --branch generic_print --branch generic_print

    git clone https://github.com/sferes2/nn2.git
    
    git clone https://github.com/resilient-swarms/cvt_map_elites.git 

5. Go back to the argos-sferes directory

    cd ../..

    mkdir build && cd build

    cmake ..
    
    make
    
6. To compile  a variety of descriptors with varying dimensions, you can use the command
   
   bash make_all_simulation.sh
   
   


Instruction for running
-------------

Edit the ARGoS experiment configuration file for your own experiment setup

    emacs argos-sferes/experiments/obsavoid_evol.argos


To run the EA, on the setting mentioned in point 5., you can run

    cd argos-sferes
     ./bin/obsavoid_evol2D experiments/history.argos

To run the EA on other settings mentioned in point 6., you can run either of the following commands:

     ./bin/obsavoid_evolcvt14D experiments/mutualinfoact.argos
     ./bin/obsavoid_evolcvt21D experiments/mutualinfo.argos
     ./bin/obsavoid_evolcvt400D experiments/spirit.argos

To try out an evolved solution

     ./bin/obsavoid_evol2D experiments/history.argos --load <path to generation file>/gen_<number> -o <output file> -n <index of individual in MAP>
