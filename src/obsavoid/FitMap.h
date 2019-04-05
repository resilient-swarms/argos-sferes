
#include <src/obsavoid/evol_loop_functions.h>



struct Params
{
    struct ea
    {
        SFERES_CONST size_t behav_dim = 7;
        SFERES_CONST double epsilon = 0;//0.05;
        SFERES_ARRAY(size_t, behav_shape, 10, 10, 10, 10, 10, 10, 10);
    };

    struct parameters
    {
        //Min and max weights of MLP?
        static constexpr float min = -5.0f;
        static constexpr float max = 5.0f;
    };

    struct evo_float
    {
        static constexpr mutation_t mutation_type = polynomial;
        //static const cross_over_t cross_over_type = sbx;
        static constexpr cross_over_t cross_over_type = no_cross_over;
        static constexpr float cross_rate = 0.0f;
        static constexpr float mutation_rate = 0.1f;
        static constexpr float eta_m = 15.0f;
        static constexpr float eta_c = 10.0f;
    };

    struct pop
    {
        // number of initial random points
        SFERES_CONST size_t init_size = 200;//1000;
        // size of a batch
        SFERES_CONST size_t size = 200; //1000;
        SFERES_CONST size_t nb_gen = 10001;
        SFERES_CONST size_t dump_period = 100;
    };
};




namespace  sferes
{
// ********** Main Class ***********
//SFERES_FITNESS(FitObstacle, sferes::fit::Fitness)

FIT_MAP(FitObstacleMapElites)
{
    public:

    FitObstacleMapElites() {
    }

    // *************** _eval ************
    //
    // This is the main function to evaluate the individual
    // It runs argos sim
    //
    // **********************************

    bool dead()
    {
        return false;
    }
    template<typename Indiv>
    void print_progress(Indiv& ind,CObsAvoidEvolLoopFunctions& cLoopFunctions, Real time)
    {
                printf("\n\n lin_speed = %f", cLoopFunctions.lin_speed);
                printf("\n\n nb_coll = %f", cLoopFunctions.nb_coll);
                int trial=cLoopFunctions.m_unCurrentTrial;
                printf("\n\n fitness in trial %lu is %f", trial,cLoopFunctions.fitfun->fitness_per_trial[trial]);

                if(trial==0)
                {
                    std::ofstream ofs("nn.dot");
                    ind.nn().write(ofs);
                }

    }
    

    

    template<typename Indiv>
    void eval(Indiv& ind)
    {
        this->_objs.resize(1);

        ind.nn().simplify();
        //ind.nn().init();


        /****************************************/
        /****************************************/
        /* The CSimulator class of ARGoS is a singleton. Therefore, to
      * manipulate an ARGoS experiment, it is enough to get its instance.
      * This variable is declared 'static' so it is created
      * once and then reused at each call of this function.
      * This line would work also without 'static', but written this way
      * it is faster. */
        static argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();

        /* Get a reference to the loop functions */
        static CObsAvoidEvolLoopFunctions& cLoopFunctions = dynamic_cast<CObsAvoidEvolLoopFunctions&>(cSimulator.GetLoopFunctions());
        for(size_t j = 0; j < cLoopFunctions.m_unNumberRobots; ++j)
            cLoopFunctions._vecctrlrob[j] = ind.nn_cpy();

        cLoopFunctions.descriptor->before_trials(cSimulator);

        /*
         * Run x trials and take the worst performance as final value.
        */
        
        for(size_t i = 0; i < cLoopFunctions.m_unNumberTrials; ++i)
        {
            cLoopFunctions.nb_coll=0;
            cLoopFunctions.stop_eval=false;
            cLoopFunctions.speed=0.0f; cLoopFunctions.lin_speed=0.0f;
            // cLoopFunctions.stand_still = 0;
            // cLoopFunctions.old_pos   = CVector3(0.0f, 0.0f, 0.0f);
            // cLoopFunctions.old_theta = CRadians(0.0f);
            cLoopFunctions.num_ds = 0.0;
            cLoopFunctions.descriptor->start_trial();

            /* Tell the loop functions to get ready for the i-th trial */
            cLoopFunctions.SetTrial(i);

            /* Reset the experiment. This internally calls also cLoopFunctions::Reset(). */
            cSimulator.Reset();

            /* Configure the controller with the indiv gen */
            //cLoopFunctions.ConfigureFromGenome(ind.nn());
            //cLoopFunctions._ctrlrob = ind.nn_cpy();
            //cLoopFunctions._ctrlrob.init(); // a copied nn object needs to be init before use


            for(size_t j = 0; j < cLoopFunctions.m_unNumberRobots; ++j)
                cLoopFunctions._vecctrlrob[j].init(); // a copied nn object needs to be init before use


            /* Run the experiment */
            cSimulator.Execute();
            Real time = (Real)cSimulator.GetMaxSimulationClock();


            
            cLoopFunctions.fitfun->apply(cLoopFunctions,time);
            #ifdef PRINTING
                
                print_progress(ind,cLoopFunctions,time);
            #endif

            cLoopFunctions.descriptor->end_trial(cLoopFunctions);



        }
        /****************************************/
        /****************************************/
        float fFitness=cLoopFunctions.fitfun->after_trials();
        this->_objs[0] = fFitness;
        this->_value   = fFitness;

        Real time=(Real)cSimulator.GetMaxSimulationClock();
        std::vector<float> behavioural_descriptor=cLoopFunctions.descriptor->after_trials(time,cLoopFunctions);
        
        this->set_desc(behavioural_descriptor);

        #ifdef PRINTING
            printf("\n\n fFitness = %f", fFitness);
        #endif

    } // *** end of eval ***
};
}

/****************************************/
/****************************************/
