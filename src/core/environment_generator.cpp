
#include <src/core/environment_generator.h>
#include <src/core/base_loop_functions.h>

void EnvironmentGenerator::generate(argos::CSimulator& cSimulator)
{

}

void ConfigurationBasedGenerator::generate(argos::CSimulator& cSimulator)
{

    cSimulator.SetExperimentFileName(filename);

    /* Load it to configure ARGoS */
    cSimulator.LoadExperiment();


}