
#include <src/core/base_loop_functions.h>
#include <src/core/environment_generator.h>


void EnvironmentGenerator::generate(BaseLoopFunctions* cLoopFunctions)
{

}

void ConfigurationBasedGenerator::generate(BaseLoopFunctions* cLoopFunctions)
{
    cLoopFunctions->Init(filename);
}