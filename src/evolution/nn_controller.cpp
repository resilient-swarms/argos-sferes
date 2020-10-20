

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
#ifdef RAB_CONTROL
    assert(nn.get_outf().size() == 3);
    m_pcRABA->ClearData();
    float m_fDataOut = nn.get_outf()[2];
    unsigned char byte = ((m_fDataOut + 1) / 2) * 255;
    m_pcRABA->SetData(0, byte);

    std::string id = GetId();
    id.erase(0,6);
    unsigned char robot_id = std::stoi(id);
    m_pcRABA->SetData(1, robot_id);
#else
    assert(nn.get_outf().size() == 2);
#endif
    m_fLeftSpeed = m_sWheelTurningParams.MaxSpeed * nn.get_outf()[0];
    m_fRightSpeed = m_sWheelTurningParams.MaxSpeed * nn.get_outf()[1];
    BaseController::ControlStep();// needed to actually move and inject faults

}

REGISTER_CONTROLLER(CThymioNNController, "nn_controller"+ std::string(TAG))