

#ifndef EVAL_PARALLEL_ENVIR_HPP
#define EVAL_PARALLEL_ENVIR_HPP

#include <cmath>
#include <sferes/eval/eval.hpp>
#include <vector>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>

#include <cstdio>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <signal.h>
#include <iostream>
#include <fstream>
#include <sys/stat.h> /* For mode constants */


#include <random>
#include <src/parallel/argosparallel_eval.h>
#include <src/evolution/descriptors.h>


namespace sferes
{
namespace eval
{ 
static std::string jobname;
static std::string redirect_dir;
class EnvirGenerator {
public:
    EnvirGenerator(){};
    EnvirGenerator(std::mt19937::result_type seed) : eng(seed) {}
    inline int generate(int options)
    { 
      return std::uniform_int_distribution<int>{1, options}(eng);
    }
private:        
    std::mt19937 eng{std::random_device{}()};
};



template <typename Phen,typename Param_t>
struct _argos_parallel_envir 
{
  typedef std::vector<boost::shared_ptr<Phen>> pop_t;
  typedef typename Phen::fit_t fit_t;
  pop_t _pop;
  fit_t _fit;
  std::vector<float> bd;
  /** PID of the master process */
  pid_t MasterPID;

  /** PIDs of the slave processes */
  std::vector<pid_t> SlavePIDs;

  ~_argos_parallel_envir(){};
  _argos_parallel_envir(pop_t &pop, const fit_t &fit) : _pop(pop),
                                                     _fit(fit),
                                                     MasterPID(::getpid())
  {
    allocate_additional_memory();
    create_processes();
    destroy_additional_memory();
  }
  _argos_parallel_envir(const _argos_parallel_envir &ev) : _pop(ev.pop),
                                                     _fit(ev.fit),
                                                     MasterPID(::getpid())
  {
    allocate_additional_memory();
    create_processes();
    destroy_additional_memory();
  }
  /* SIGTERM handler for slave processes */
  // inline void SlaveHandleSIGTERM(int) {
  //   argos::CSimulator::GetInstance().Destroy();
  //   argos::LOG.Flush();
  //   argos::LOGERR.Flush();
  //   CleanUp();
  // }
  // inline void CleanUp()
  // {
  //   for(UInt32 i = 0; i < m_unPopSize; ++i) {
  //     ::kill(SlavePIDs[i], SIGTERM);
  //   }

  // }
  void allocate_additional_memory()
  {
     size_t to_add = _pop.size() - num_memory;
     
     for (size_t i=0; i< to_add; ++i)
     {
       shared_memory.push_back(new CSharedMem(BEHAV_DIM));
     }
     //std::cout<<"allocated memory: "<<shared_memory.size()<<std::endl;// this should happen only at the 0'th generation
  }

  void destroy_additional_memory()
  {
    
    shared_memory.erase(shared_memory.begin()+num_memory,shared_memory.end());
    //std::cout<<"erased memory: "<<shared_memory.size()<<std::endl;// this should happen only at the 0'th generation
  }

  void wait_and_erase()
  {
    // wait for a new process to finish and erase it from the list
      int *status;
      pid_t pid = waitpid(-1,status,0);// -1: any child; status; 0: only children that exit
      //std::cout<<"waited for pid "<<pid<<std::endl;
      auto it = std::find(SlavePIDs.begin(),SlavePIDs.end(),pid);
      SlavePIDs.erase(it);// remove the process from the list
  }

  /* create the different child processes */
  void create_processes()
  {

    /* Create and run slave processes until all individuals done; but never start more then NUM_CORES processes */
    size_t i = 0;
    while (i < _pop.size() )
    {
      //std::cout<<"count "<< i << std::endl;
      /* initialise the fitmap */
      _pop[i]->fit() = _fit;

      if (SlavePIDs.size() < NUM_CORES)  // still need more
      {
        /* Perform fork */
        SlavePIDs.push_back(::fork());
        if (SlavePIDs.back() == 0)
        {
          /* We're in a slave */
        this->LaunchSlave(i);
        }
        else{
           ++i;// increment the parent's index
        }
       
      }
      else{
        wait_and_erase();
      }
      
      
    }
    // now wait for all the other child processes to finish
    while (!SlavePIDs.empty())
    {
        wait_and_erase();
    };// keep performing waits until an error returns, meaning no more child existing

    /* Back in the parent, copy the scores into the population data */
    for (size_t i = 0; i < _pop.size(); ++i)
    {
      _pop[i]->fit().set_fitness(shared_memory[i]->getFitness());
      bd = shared_memory[i]->getDescriptor();
      _pop[i]->fit().set_desc(bd);
      _pop[i]->fit().set_dead(shared_memory[i]->getDeath());
      // argos::LOG << "parent fitness " << i << " " << _pop[i]->fit().obj(0) << std::endl;
      //  argos::LOG << "parent: descriptor for individual " << i << std::endl;
      //  for (size_t j = 0; j < _pop[i]->fit().desc().size(); ++j)
      //  {
      //   argos::LOG << "   " << _pop[i]->fit().desc()[j] << std::endl;
      //  }
      //  argos::LOG << "parent: death " << _pop[i]->fit().dead() << std::endl;
    }
    argos::LOG.Flush();
    argos::LOGERR.Flush();
    //argos::LOG << "finished all processes "<< std::endl;
  }
  inline float bin_option(size_t option,size_t num_options)
  {
    //place them in the bottom of the bin e.g[0,5) [0.5,1)
    return (float)(option - 1) / (float)num_options; // cf. behav_pos[i] = round(p[i] * behav_shape[i]); l.192-193 map_elites.hpp
  }

  void LaunchSlave(size_t slave_id)
  {
      /* note: needs to be in a cpp ?*/
      /* Initialize ARGoS */
      /* Redirect LOG and argos::LOGERR to dedicated files to prevent clutter on the screen */
      /* append to the back of the file */
      std::ofstream cLOGFile("/dev/null", std::ios::app);
      argos::LOG.DisableColoredOutput();
      argos::LOG.GetStream().rdbuf(cLOGFile.rdbuf());
      std::ofstream cLOGERRFile(std::string(redirect_dir + "/LOGERR_" + argos::ToString(getppid()) + "_" + std::to_string(slave_id)).c_str(), std::ios::app);
      argos::LOGERR.DisableColoredOutput();
      argos::LOGERR.GetStream().rdbuf(cLOGERRFile.rdbuf());

      /* The CSimulator class of ARGoS is a singleton. Therefore, to
      * manipulate an ARGoS experiment, it is enough to get its instance */
      argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();
      try
      {
          EnvirGenerator g = EnvirGenerator(time(NULL) * getpid());
          jobname = jobname + "_";
          int option;
          /* describe the environment */
          std::vector<float> bd;
          for (int i = 0; i < Param_t::options.size() - 1; ++i)
          {
              option = g.generate(Param_t::options[i]);
              jobname = jobname + std::to_string(option) + ",";
              bd.push_back(bin_option(option,Param_t::options[i]));
              
          }
          option = g.generate(Param_t::options.back());
          bd.push_back(bin_option(option,Param_t::options.back()));
          jobname = jobname + std::to_string(option) + ".argos";
          //argos::LOG << "loading " << jobname << std::endl;
          // /* Set the .argos configuration file
          //  * This is a relative path which assumed that you launch the executable
          //  * from argos3-examples (as said also in the README) */
          cSimulator.SetExperimentFileName(jobname);
          // /* Load it to configure ARGoS */
          cSimulator.LoadExperiment();

          static BaseEvolutionLoopFunctions &cLoopFunctions = dynamic_cast<BaseEvolutionLoopFunctions &>(cSimulator.GetLoopFunctions());
          cLoopFunctions.descriptor = new StaticDescriptor(bd);
      }
      catch (argos::CARGoSException &ex)
      {
          argos::LOGERR << ex.what() << std::endl;
          argos::LOGERR.Flush();
          ::raise(SIGKILL);
      }
      //argos::LOG << "child starting " << std::endl;
      // initialise the fitness function and the genotype
      _pop[slave_id]->develop();
      assert(slave_id < _pop.size());
      // evaluate the individual
      _pop[slave_id]->fit().eval(*_pop[slave_id]);

      assert(!std::isnan(_pop[slave_id]->fit().objs()[0])); // ASSUMES SINGLE OBJECTIVE
      // write fitness and descriptors to shared memory
      shared_memory[slave_id]->setFitness(_pop[slave_id]->fit().objs()[0]); // ASSUME SINGLE OBJECTIVE
      shared_memory[slave_id]->setDescriptor(_pop[slave_id]->fit().desc());
      shared_memory[slave_id]->setDeath(_pop[slave_id]->fit().dead());
      // argos::LOG << "child fitness " << slave_id << " " << _pop[slave_id]->fit().obj(0) << std::endl;
      // argos::LOG << "child: descriptor for individual " << slave_id << std::endl;

      // for (size_t j = 0; j < _pop[slave_id]->fit().desc().size(); ++j)
      // {
      //   argos::LOG << "   " << _pop[slave_id]->fit().desc()[j] << std::endl;
      // }
      // argos::LOG << "child: death " << _pop[slave_id]->fit().dead() << std::endl;
      argos::LOG.Flush();
      argos::LOGERR.Flush();
      cSimulator.Destroy(); // difference to the usual argosparallel
      exit(EXIT_SUCCESS);
    }

};

SFERES_EVAL(ArgosParallelEnvir, Eval){
  public :
      template <typename Phen>
      void eval(std::vector<boost::shared_ptr<Phen>> & pop, size_t begin, size_t end,
                const typename Phen::fit_t &fit_proto){
            dbg::trace trace("eval", DBG_HERE);
          assert(pop.size());
          assert(begin < pop.size());
          assert(end <= pop.size());
#ifdef ANALYSIS
          throw std::runtime_error("cannot use parallel while doing analysis");
#endif
          /* if you want timer */
          //auto t1 = std::chrono::system_clock::now();
          
          
          _argos_parallel_envir<Phen,Params>(pop, fit_proto);
          
          /* stop timer */
          // Some computation here
          //auto t2 = std::chrono::system_clock::now();
          //std::chrono::duration<double> duration =t2 - t1;
          //argos::LOG <<"evaluation time: "<< duration.count() <<'\n';
          //argos::LOG.Flush();

          this->_nb_evals += (end - begin);
      } // namespace eval

}; // namespace sferes

}
}



#endif
