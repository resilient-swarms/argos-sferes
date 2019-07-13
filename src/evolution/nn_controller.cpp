

#include <src/evolution/nn_controller.h>

std::vector<float> CThymioNNController::InputStep()
{

    std::vector<float> in;
    if (only_proximity)
    {
        // NOTE: do not use normalisation either
        for (size_t i = 0; i < m_pcProximity->GetReadings().size(); ++i)
        {
            in.push_back(m_pcProximity->GetReadings()[i].Value);
        }
    }
    else
    {
        std::vector<Real> readings = GetNormalizedSensorReadings();
        in = std::vector<float>(readings.begin(), readings.end());
    }
    in.push_back(+1.0); //Bias input

    return in;
}

void CThymioNNController::ControlStep()
{
    //assert(cController.m_pcProximity->GetReadings().size() + 1 == Params::dnn::nb_inputs); //proximity sensors + bias  given as input to nn
    inputs = InputStep();

    //      _ctrlrob.step(inputs);
    nn.step(inputs);
    nn.get_outf();

    assert(nn.get_outf().size() == 2);
    m_fLeftSpeed = m_sWheelTurningParams.MaxSpeed * nn.get_outf()[0];
    m_fRightSpeed = m_sWheelTurningParams.MaxSpeed * nn.get_outf()[1];
    BaseController::ControlStep();// needed to actually move
}

REGISTER_CONTROLLER(CThymioNNController, "nn_controller")