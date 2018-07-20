argos-sferes
=======

An interface for using the ARGoS swarm robotics simulator with the Sferes evolutionary algorithm framework.


Instruction for compilation (using MAP-Elites and neural-network modules)
-------------

Install ARGoS https://www.argos-sim.info/ (via binary packages or source)

Download argos-sferes code
git clone https://github.com/daneshtarapore/argos-sferes.git

Now download the sferes2 code -- all header files, so don't need compilation (unless you want to run the sferes2 test cases).
cd argos-sferes
git clone https://github.com/sferes2/sferes2.git 

As nn2 and map_elites code are in header files, they don't need to be compiled -- so no need to add them to modules.conf
cd sferes2/modules
git clone https://github.com/sferes2/map_elites.git --branch generic_print
git clone https://github.com/sferes2/nn2.git

Go back to the argos-sferes directory
cd ../..
mkdir build
cmake ..


Instruction for running
-------------

Edit the ARGoS experiment configuration file for your own experiment setup
emacs argos-sferes/experiments/evolution.argos

To run the EA
argos-sferes/bin/argos_galib

To try out an evolved solution
argos-sferes/bin/argos_galib --load <path to generation file>/gen_<number> -o <output file> -n <number of individual in MAP>
