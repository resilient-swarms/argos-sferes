

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
    else{
        std::vector<Real> readings = GetNormalizedSensorReadings();
        in = std::vector<float>(readings.begin(), readings.end());
    }
    in.push_back(+1.0); //Bias input

    return in;
}





REGISTER_CONTROLLER(CThymioNNController, "nn_controller")