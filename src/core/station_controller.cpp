
#include <src/core/station_controller.h>

CThymioStationController::CThymioStationController()
{
    //m_fDataOut = 0;
}

void CThymioStationController::ControlStep()
{
    m_fLeftSpeed = 0;
    m_fRightSpeed = 0;
    BaseController::ControlStep();// needed to actually move

    m_pcRABA->ClearData();
    m_pcRABA->SetData(0, 0);

    std::string id = GetId();
    id.erase(0,6);
    unsigned char robot_id = std::stoi(id);
    m_pcRABA->SetData(1, robot_id);
}

REGISTER_CONTROLLER(CThymioStationController, "station_controller")