//
//  ElectricalVehicle.h
//  MHP-LibLinearOptimizer
//
//  Created by Eric Middelhove on 27.06.23.
//

#ifndef ElectricalVehicle_h
#define ElectricalVehicle_h

#include <stdio.h>
class ElectricalVehicle {

public:
    /**
     * @brief Construct a new Electrical Vehicle object
     *
     * @param minimumChargingPower  The minimum charging power the Vehicle needs to charge
     * @param maximumChargingPower The maximum charging power the Vehicle is able to use
     * @param batteryCapacity The capacity of the vehicles battery
     * @param percentCharged  The percentage the battery of the vehicle is charged
     * @param amount_of_15_minutes The lenght of the solution array
     */
    ElectricalVehicle(unsigned short minimumChargingPower, unsigned short maximumChargingPower, unsigned long batteryCapacity, unsigned long batteryContent, unsigned long batteryTargetCapacity, int intervals_staying, size_t arrival_interval, const char name[]);
    ElectricalVehicle();

    ~ElectricalVehicle();

    unsigned short minimumChargingPower;
    unsigned short maximumChargingPower;        // kW
    unsigned long batteryCapacity;              // kWh
    unsigned long batteryContent;               // State of Charge
    unsigned long batteryTargetCapacity;                      // Maximum State of Charge
    bool isCharging;
    size_t arrival_interval;
    size_t intervals_staying;                   // alias to minimumChargingTime but in intervals not in minutes

    const char *name;

    double* chargingplan;               // array of length amount_15_minutes that will contain the charginplan for this particular vehicle

    void allocateChargingPlan(size_t length);

    void log();

private:
    void freeChargingPlan();
    bool chargingplan_was_allocated = false;
};

#endif /* ElectricalVehicle_h */
