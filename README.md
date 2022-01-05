argos-sferes
=======

An interface for using the ARGoS swarm robotics simulator with the Sferes evolutionary algorithm framework.

The argos-sferes repository has been used in the papers:

Bossens, D. M., & Tarapore, D. (2020). QED: using Quality-Environment-Diversity to evolve resilient robot swarms. IEEE Transactions on Evolutionary Computation. https://doi.org/10.1109/TEVC.2020.3036578  

NOTE: see also [QED VIDEO from argos-sferes](https://www.youtube.com/watch?v=BN6i-NugCGg&t=134s)

Bossens, D. M., & Tarapore, D. (2021). Rapidly adapting robot swarms with Swarm Map-based Bayesian Optimisation. The 2021 International Conference on Robotics and Automation (ICRA 2021). [arXiv link at http://arxiv.org/abs/2012.11444; IEEE Explore link at https://ieeexplore.ieee.org/abstract/document/9560958]

NOTE: see also [SMBO VIDEO from argos-sferes](https://www.youtube.com/watch?v=I1HcRn0pOf4)



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
    
    git clone https://github.com/resilient-swarms/limbo.git
    



5. Compilation of evolution experiments:


(a) To compile serial evolution experiments:

   
    bash cmake_scripts/make_all_simulation.sh
    

(b) To compile parallel evolution experiments:

    
    bash cmake_scripts/make_all_parallel.sh
    

(c) To compile evolution experiments with parallel environments:



    bash cmake_scripts/make_all_envirparallel.sh
   
    

6. To compile baseline behaviours useful for comparison on Dispersion, Aggregation, etc.:



    bash make_baseline.sh
    

7. Compilation of Swarm Map-based Bayesian Optimisation (SMBO):


    bash cmake_scripts/make_all_BO.sh
    
    
8. Compilation of SMBO-Decentralised:

(a) To support heterogeneous swarms in a single run, one needs to compile a binary for printing the correct network to file:


   bash cmake_scripts/make_all_printnet.sh


(b) To compile SMBO-Dec on a small foraging environment:


   
    bash cmake_scripts/make_all_heterosim.sh


(c) To compile SMBO-Dec on a larger foraging environment:

   
    bash cmake_scripts/make_all_largeheterosim.sh


(d) To compile SMBO-Dec on a larger foraging environment:

   
    bash cmake_scripts/make_all_largeheterosim.sh



(e) To compile a few binaries based on different acquisition functions and hyperparameters:

    
    bash cmake_scripts/make_all_heteroexps.sh ${acq} ${kern} ${alpha} ${lengthscale}


where ${acq} is the acquisition function in {0 (=UCB),1 (=UCB_LOCAL),2 (=UCB_LOCAL2),3 (=UCB_LOCAL3)}, ${kern} is the kernel in {0 (Matern52 with variable noise), 2 (Matern52)}, $alpha is the exploration exploitation parameter for UCB, and ${lengthscale} is the lengthscale of the kernel (see more info below in Compilation macros). 


9. Compile performance recording of a particular swarm:

    bash cmake_scripts/make_all_recording.sh


    

Compilation macros for further customisation
-------------


It is best to start of with some of the above scripts and then modify their parameters for customisation. The following macros can be used for further customisation of the compilation scripts:

    * `NN_INPUT_TYPE`: type of neural network control
      - 0: NN control based on proximity and RAB sensors    (all tasks except Foraging)
      - 1: NN control based on proximity and ground sensors (Foraging task)
      - 2: NN with RAB control (experimental; all tasks except Foraging)

    * `BD`: integer denoting the number of dimensions in the behaviour space

    * `CVT_USAGE`: whether or not to use CVT to organise behaviour space
	-ON: use CVT  (for high-dimensional behaviour spaces)
	-OFF: do not use CVT (for low-dimensional behaviour spaces)

    * `ARGOS_PAR`: serial or parallel evolution (only for evolution, not adaptation)
      - 0: serial evolution
      - 1: parallelisation of individuals in the population of an evolutionary algorithm
      - 2: parallelisation of environments with Quality-Environment-Diversity evolution (see reference mentioned at start of README)
    
    * `NUM_CORES`: integer denoting the number of cores (1 by default; only for evolution)

    * `LARGE`: size of arena (only for Foraging task)
      - "ON": 4.2m x 2.1m Foraging arena
      - "OFF": 2.1m x 2.1m Foraging arena

    * `BO_ACQ`: set the type of acquisition function in SMBO (only for adaptation)
      - 0: traditional UCB
      - 2: UCB with soft local penalisation
      - 3: UCB with hard local penalisation
      - 4: UCB with hard local penalisation and local Lipschitz constants


    * `BO_KERN`: the type of kernel     (only for adaptation)
      - 0: Matern 5/2 with variable noise
      - 2: Matern 5/2 with fixed noise

    * `LIMBO_ALPHA`: positive float denoting the exploration-exploitation tradeoff parameter in UCB  (only for adaptation)

    * `LIMBO_L`: positive float denoting the lengthscale parameter in UCB   (only for adaptation)


Instruction for running
-------------


Evolution in the normal operating environment can be run using:

   
     ./bin/behaviour_evolcvt10D experiments/Gomes_walls_and_robots_std.argos
   
 
Note here the binaries are organised as "behaviour_evol"${CVT}${DIM}D" where CVT is whether or not using CVT-MAPElites or MAPElites and DIM is the dimensionality of the behavioural descriptor.


Hand-crafted behaviours can also be experimented with just to see how well evolution compares:



     ./bin/baseline_behaviour experiments/baseline-behavs.argos
   


Running the experiments from the QED paper (Aggregation, Dispersion, Flocking, Patrolling, Border-patrolling) can be done using the "launch" scripts:


For QED, this is:

      bash launch_environment_descriptor.sh ${data_directory}
   
   
For the baseline QD-algorithms, this is:


      bash launch_Gomes_jobs.sh ${data_directory}


To try out an evolved solution

     ./bin/behaviour_evol2D experiments/history.argos --load <path to generation file>/gen_<number> -o <output file> -n <index of individual in MAP>

To try out Bayesian Optimisation for a homogeneous swarm, do:

     bin/ite_swarms_3D -m <data_directory> 20000 -e bin/BO3D 
     experiments/history_BO.argos

Finally, to set up SMBO-Dec for Bayesian Optimisation with a heterogeneous swarm (on the Foraging tasks), the launch script may be most useful due to the many arguments:

     bash launch_foraging_BO_array.sh <data_directory>

A wide variety of other launch scripts can also be found in the main directory, each of them prefixed by "launch_"
