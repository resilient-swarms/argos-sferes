

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




namespace sferes
{
namespace eval
{ 
static std::string jobname;

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



template <typename Phen>
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
    create_processes();
  }
  _argos_parallel_envir(const _argos_parallel_envir &ev) : _pop(ev.pop),
                                                     _fit(ev.fit),
                                                     MasterPID(::getpid())
  {
    create_processes();
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

  void LaunchSlave(size_t slave_id);
  /* create the different child processes */
  void create_processes()
  {

    /* Create slave processes */
    for (size_t i = 0; i < _pop.size(); ++i)
    {

      /* initialise the fitmap */
      _pop[i]->fit() = _fit;

      /* Perform fork */
      SlavePIDs.push_back(::fork());
      if (SlavePIDs.back() == 0)
      {
        /* We're in a slave */
       this->LaunchSlave(i);
      }
    }
    /* Back in the parent, copy the scores into the population data */
    for (size_t i = 0; i < _pop.size(); ++i)
    {
      siginfo_t siginfo;
      pid_t pid = SlavePIDs[i];
      ::waitid(P_PID, pid, &siginfo, WEXITED);// wait until the child finishes
      //argos::LOG << "parent finished waiting " << pid << std::endl;
      _pop[i]->fit().set_fitness(shared_memory[i]->getFitness());
      bd = shared_memory[i]->getDescriptor();
      _pop[i]->fit().set_desc(bd);
      _pop[i]->fit().set_dead(shared_memory[i]->getDeath());
      // argos::LOG << "parent fitness " << i << " " << _pop[i]->fit().obj(0) << std::endl;
      // argos::LOG << "parent: descriptor for individual " << i << std::endl;
      // for (size_t j = 0; j < _pop[i]->fit().desc().size(); ++j)
      // {
      //   argos::LOG << "   " << _pop[i]->fit().desc()[j] << std::endl;
      // }
      // argos::LOG << "parent: death " << _pop[i]->fit().dead() << std::endl;
    }
    argos::LOG.Flush();
    argos::LOGERR.Flush();
    SlavePIDs.clear();     // PIDs no longer exist
    //argos::LOG << "finished all processes "<< std::endl;
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

          _argos_parallel_envir<Phen>(pop, fit_proto);

          this->_nb_evals += (end - begin);
      } // namespace eval

}; // namespace sferes

}
}



#endif
