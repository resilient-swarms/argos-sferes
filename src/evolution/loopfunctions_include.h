//put this file in your scripts

#if NN_DIM_TYPE == 1
#include <src/evolution/foraging_loop_functions.h>
typedef CForagingLoopFunctions MainLoopFunctions;
#else
#include <src/evolution/evol_loop_functions.h>
typedef EvolutionLoopFunctions MainLoopFunctions;
#endif

#include <chrono>
using namespace std::chrono;

namespace sferes
{
    // ********** Main Class ***********
    //SFERES_FITNESS(FitObstacle, sferes::fit::Fitness)

    FIT_MAP(FitObstacleMapElites)
    {

    public:
        FitObstacleMapElites() {}
        bool died = false;
        inline bool dead()
        {
            return died;
        }

        // *************** _eval ************
        //
        // This is the main function to evaluate the individual
        // It runs argos sim
        //
        // **********************************
        inline void set_fitness(float fFitness)
        {
            if (died)
                fFitness = 0.0f;
            this->_objs.resize(1);
            this->_objs[0] = fFitness;
            this->_value = fFitness;
        }
        inline void set_dead(bool dead)
        {
            this->died = dead;
        }
        template <typename Indiv>
        void eval(Indiv & ind)
        {
            /* if you want timer */
            //std::clock_t start = std::clock();
            ind.nn().simplify();
            //ind.nn().init();
            /* The CSimulator class of ARGoS is a singleton. Therefore, to
            * manipulate an ARGoS experiment, it is enough to get its instance.
            * This variable is declared 'static' so it is created
            * once and then reused at each call of this function.
            * This line would work also without 'static', but written this way
            * it is faster. 
            */

            static argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();
            static MainLoopFunctions &cLoopFunctions = dynamic_cast<MainLoopFunctions &>(cSimulator.GetLoopFunctions());
#if !HETEROGENEOUS
            for (size_t j = 0; j < cLoopFunctions.m_unNumberRobots; ++j)
                cLoopFunctions.m_pcvecController[j]->nn = ind.nn_cpy();
                // auto start = high_resolution_clock::now();
#if PRINTING == 1 || BAYESIAN_OPT == 1 || PRINT_NETWORK == 1

            std::cout << "writing individual to .dot file" << std::endl;
            std::ofstream ofs("nn.dot");
            ind.nn().write(ofs);
            std::cout << "Finish writing network" << std::endl;
            NNSerialiser ser = NNSerialiser(cLoopFunctions.savefile);
            ser.Save<Indiv>(ind);
#ifdef PRINT_NETWORK // use either just to print a network or else not interested in the full trial's outcome
            return;
#endif

#endif
#endif
            float fFitness = cLoopFunctions.run_all_trials(cSimulator);
            set_dead(cLoopFunctions.stop_eval);
            set_fitness(fFitness);
            //std::cout<<"fitness: "<<fFitness<<std::endl;

#ifdef ANALYSIS
#if ARGOS_PARALLEL > 0 // if parallel then will be using the shared memory initialisations, so evolution would not give a segmentation fault
            kill(getppid(), 9);
            throw std::runtime_error("evol: cannot use parallel while doing analysis");

#endif
            cLoopFunctions.analyse(fFitness);
            return;
#endif

#ifndef BAYESIAN_OPT
            std::vector<float> behavioural_descriptor = cLoopFunctions.alltrials_descriptor();

            this->set_desc(behavioural_descriptor);
#endif

#ifdef PRINTING

            printf("\n\n fFitness = %f", fFitness);
#endif
            // auto stop = high_resolution_clock::now();
            // auto duration = duration_cast<milliseconds>(stop - start);
            // std::cout << "evaluation took " << duration.count() << " ms"<<std::endl;

            /* stop timer */
            //double duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

            // argos::LOG <<"evaluation time: "<< duration <<'\n';
            // argos::LOG.Flush();
        } // *** end of eval ***
    };
} // namespace sferes
