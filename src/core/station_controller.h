#ifndef STATION_CONTROLLER_H
#define STATION_CONTROLLER_H

#include <src/core/base_controller.h>

class CThymioStationController : public BaseController
{
public:
    CThymioStationController();

    virtual void ControlStep();
};


#endif //STATION_CONTROLLER_H
