#include <vector>
#include <src/core/base_controller.h>

class CThymioNNController : public BaseController
{
public:
    /* get the inputs as a vector of floats*/
    std::vector<float> InputStep();

    CThymioNNController(){};
};