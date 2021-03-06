//| This file is a part of the sferes2 framework.
//| Copyright 2009, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s): Jean-Baptiste Mouret, mouret@isir.fr
//|
//| This software is a computer program whose purpose is to facilitate
//| experiments in evolutionary computation and evolutionary robotics.
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software.  You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.
#ifndef EVAL_PARALLEL2_HPP
#define EVAL_PARALLEL2_HPP

#include <sferes/parallel.hpp>
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

namespace sferes
{
namespace eval
{
static size_t num_processes;
static size_t num_individuals_per_process;
/* File name for shared memory area (use for communicating genome) */
//static const std::string SHARED_MEMORY_FILE = "/MPGA_SHARED_MEMORY_" + argos::ToString(getpid());
/** Shared memory manager for data exchange between master and slaves */
class CSharedMem
{

public:
  /**
       * Class constructor.
       * @param un_genome_size The size of the genome of an individual.
       * @param un_pop_size The size of the population.
       */
  CSharedMem(size_t descriptor_size) : m_unDescriptorSize(descriptor_size)
  {
    init_shared_mem();
  }
  void init_shared_mem()
  {
    size_t unShareMemSize = get_block_size() * sizeof(float); //m_unPopSize *
    /* Get pointer to shared memory area */
    // Memory buffer will be readable and writable:
    int protection = PROT_READ | PROT_WRITE;
    // The buffer will be shared (meaning other processes can access it), but
    // anonymous (meaning third-party processes cannot obtain an address for it),
    // so only this process and its children will be able to use it:
    int visibility = MAP_ANONYMOUS | MAP_SHARED;
    m_pfSharedMem = reinterpret_cast<float *>(::mmap(NULL, unShareMemSize, protection, visibility, -1, 0));
    if (m_pfSharedMem == MAP_FAILED)
    {
      ::perror("shared memory");
      exit(1);
    }
  }

  void init_shared_mem_filedescriptor()
  {
    /* Create shared memory area for master-slave communication */
    // m_nSharedMemFD = ::shm_open(SHARED_MEMORY_FILE.c_str(),
    //                      O_RDWR | O_CREAT,
    //                      S_IRUSR | S_IWUSR);
    // if(m_nSharedMemFD < 0) {
    //     ::perror(SHARED_MEMORY_FILE.c_str());
    //     exit(1);
    // }
    // size_t unShareMemSize = m_unPopSize * get_block_size() * sizeof(float);
    //::ftruncate(m_nSharedMemFD, unShareMemSize);
    /* Get pointer to shared memory area */
    // m_pfSharedMem = reinterpret_cast<float*>(
    //     ::mmap(NULL,
    //           unShareMemSize,
    //           PROT_READ | PROT_WRITE,
    //           MAP_SHARED,
    //           m_nSharedMemFD,
    //           0));
    // if(m_pfSharedMem == MAP_FAILED) {
    //     ::perror("shared memory");
    //     exit(1);
    // }
  }
  /**
       * Class destructor.
       */
  ~CSharedMem()
  {
    munmap(m_pfSharedMem, get_block_size() * sizeof(float));
  }

  /* get the memory block size */

  inline size_t get_block_size()
  {
    return (m_unDescriptorSize + 1);
  }
  /**
       * Returns the score of an individual.
       * @param un_individual The individual.
       */
  inline float getFitness()
  {
    return m_pfSharedMem[0];
  }
  /**
       * Returns the descriptor of an individual.
       * @param un_individual The individual.
       */
  inline std::vector<float> getDescriptor()
  {
    float *descriptor = &m_pfSharedMem[1]; // pointer to the descriptor
    std::vector<float> bd;
    for (int i = 0; i < BEHAV_DIM; ++i)
    {
      bd.push_back(descriptor[i]);
    }
    return bd;
  }
  /**
       * Sets the score of an individual.
       * @param un_individual The individual.
       * @param f_score The score.
       */
  inline void setFitness(float f_score)
  {
    /* fitness is copied to the 0'th index of each block */
    ::memcpy(m_pfSharedMem + 0,
             &f_score,
             sizeof(float));
  }

  /**
       * Sets the descriptor of an individual.
       * @param un_individual The individual.
       * @param desc The descriptor
       */
  inline void setDescriptor(std::vector<float> desc)
  {
    /* descriptor is copied to the 1:m_unDescriptorSize'th index of each block */
    for (int i = 0; i < desc.size(); ++i)
    {
      ::memcpy(m_pfSharedMem + (i + 1),
               &desc[i],
               sizeof(float));
    }
  }

private:
  /** Descriptor size */
  size_t m_unDescriptorSize;

  // /** Population size */
  // size_t m_unPopSize;

  /** File descriptor for shared memory area */
  //int m_nSharedMemFD;

  /** Pointer to the shared memory area */
  float *m_pfSharedMem;
};
template <typename Phen>
struct _parallel_evaluate
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
  /* argos config */
  std::string m_strARGoSConf;
  /** The shared memory manager */
  std::vector<CSharedMem *> m_pcSharedMem;
  ~_parallel_evaluate() {}
  _parallel_evaluate(pop_t &pop, const fit_t &fit) : _pop(pop),
                                                     _fit(fit),
                                                     MasterPID(::getpid())
  {
    create_processes();
  }
  _parallel_evaluate(const _parallel_evaluate &ev) : _pop(ev.pop),
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
  inline void LaunchSlaveMultiple(size_t start_id)
  {
     /* We're in a slave */
      for (size_t j = 0; j < num_individuals_per_process; ++j)
      {   

        LaunchSlave(start_id + j);
      } 
      exit(EXIT_SUCCESS);
  }
  void LaunchSlave(size_t slave_id)
  {
    
    //pid_t slave_pid = SlavePIDs[slave_id];
    /* Install handler for SIGTERM */
    //::signal(SIGTERM, SlaveHandleSIGTERM);

    // initialise the fitness function and the genotype
    /* initialise the fitmap */
    _pop[slave_id]->fit() = _fit;
    _pop[slave_id]->develop();
    assert(i < _pop.size());
    // evaluate the individual
    _pop[slave_id]->fit().eval(*_pop[slave_id]);

    assert(!std::isnan(_pop[slave_id]->fit().objs()[0])); // ASSUMES SINGLE OBJECTIVE
    // write fitness and descriptors to shared memory
    m_pcSharedMem[slave_id]->setFitness(_pop[slave_id]->fit().objs()[0]); // ASSUME SINGLE OBJECTIVE
    m_pcSharedMem[slave_id]->setDescriptor(_pop[slave_id]->fit().desc());

    //argos::LOG << "child fitness " << slave_id << " " <<_pop[slave_id]->fit().obj(0) << std::endl;
    //argos::LOG <<"child: descriptor for individual "<< slave_id << std::endl;
    //for (size_t j=0; j < _pop[slave_id]->fit().desc().size()  ;++j)
    //{
    //  argos::LOG << "   "<<_pop[slave_id]->fit().desc()[j] << std::endl;
    //}
    //argos::LOG.Flush();
    //exit(EXIT_SUCCESS);// if you would run only one individual per slave
  }
  /* create the different child processes */
  void create_processes()
  {
    size_t i = 0;
    /* Create slave processes */
    for (size_t p = 0; p < num_processes; ++i)
    {
      /* Create shared memory manager */
      m_pcSharedMem.push_back(new CSharedMem(BEHAV_DIM));
      /* Perform fork */
      SlavePIDs.push_back(::fork());
      if (SlavePIDs.back() == 0)
      {
          LaunchSlaveMultiple(i);// launch slave for individual i
      }
      else{
        i += num_individuals_per_process;// continue with population index where the current child should finish
      }
    }
    /* Back in the parent, copy the scores into the population data */
    for (size_t i = 0; i < _pop.size(); ++i)
    {
      siginfo_t siginfo;
      size_t process_index = i/num_individuals_per_process;
      pid_t pid = SlavePIDs[process_index];
      while (i%num_individuals_per_process==0 && ::waitid(P_PID, pid, &siginfo, WEXITED) > 0)
      {
      } // wait until the child finishes (i%num_ind.. means only do it once per child otherwise already exited)
      //argos::LOG << "parent finished waiting " << pid << std::endl;
      _pop[i]->fit() = _fit;
      _pop[i]->fit().set_fitness(m_pcSharedMem[i]->getFitness());
      bd = m_pcSharedMem[i]->getDescriptor();
      _pop[i]->fit().set_desc(bd);

      //argos::LOG << "parent fitness " << i << " " <<_pop[i]->fit().obj(0) << std::endl;
      //argos::LOG <<"parent: descriptor for individual "<< i << std::endl;
      //for (size_t j=0; j < _pop[i]->fit().desc().size()  ;++j)
      //{
      //  argos::LOG << "   "<<_pop[i]->fit().desc()[j] << std::endl;
      //}
    }

    m_pcSharedMem.clear(); // destroys all the shared memory (don't do it one by one because based on pointers)
    SlavePIDs.clear();     // PIDs no longer exist
    //argos::LOG << "finished all processes "<< std::endl;
  }
};

SFERES_EVAL(ArgosParallel, Eval){
  public :
      template <typename Phen>
      void eval(std::vector<boost::shared_ptr<Phen>> & pop, size_t begin, size_t end,
                const typename Phen::fit_t &fit_proto){
          dbg::trace trace("eval", DBG_HERE);
assert(pop.size());
assert(begin < pop.size());
assert(end <= pop.size());

_parallel_evaluate<Phen>(pop, fit_proto);

this->_nb_evals += (end - begin);
} // namespace eval

}; // namespace sferes

} // namespace sferes
}


#endif