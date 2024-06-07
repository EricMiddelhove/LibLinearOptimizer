#ifndef MHP_LIBLINEAROPTIMIZER_LIBRARY_H
#define MHP_LIBLINEAROPTIMIZER_LIBRARY_H

//
//  main.hpp
//  libchargingplanoptimiser
//
//  Created by Eric Middelhove on 28.06.23.
//

#include <stdlib.h>
#include <stdio.h>
#include <vector>

#include "ElectricalVehicle.h"

char optimise(
        ElectricalVehicle EV_arr[],
        size_t evCount,
        unsigned short emissionWeight,
        unsigned short energyWeight,
        const int dynamicConsumption[],
        const unsigned int optimisationFrameSize,
        bool optimise,
        const double pvPowerRooftop_arr[],
        const double energyCosts_arr[],
        const int intervalDuration,
        const int baseMaxBuildingCapacity
);

#endif //MHP_LIBLINEAROPTIMIZER_LIBRARY_H
