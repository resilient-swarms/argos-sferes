#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <vector>
#include <src/core/base_controller.h>
#include <src/evolution/params.h>




class CThymioNNController : public BaseController
{
public:
    std::vector<float> inputs;
    /* get the inputs as a vector of floats*/
    std::vector<float> InputStep();
    robots_nn::nn_t nn;
    CThymioNNController(){};


    virtual void ControlStep();
};


#endif