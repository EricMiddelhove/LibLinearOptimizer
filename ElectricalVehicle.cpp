//
//  ElectricalVehicle.cpp
//  MHP-LibLinearOptimizer
//
//  Created by Eric Middelhove on 27.06.23.
//

#include "ElectricalVehicle.h"

ElectricalVehicle::ElectricalVehicle(unsigned short minimumChargingPower, unsigned short maximumChargingPower, unsigned long batteryCapacity, unsigned long batteryContent, unsigned long batteryTargetCapacity, int intervals_staying, size_t arrival_interval, const char name[]) {
    this->minimumChargingPower = minimumChargingPower;
    this->maximumChargingPower = maximumChargingPower;
    this->batteryContent = batteryContent;
    this->batteryCapacity = batteryCapacity;
    this->batteryTargetCapacity = batteryTargetCapacity;
    this->intervals_staying = intervals_staying;
    this->arrival_interval = arrival_interval;
    this->name = name;

}
ElectricalVehicle::ElectricalVehicle() {
    this->minimumChargingPower = 0;
    this->maximumChargingPower = 0;
    this->batteryContent = 0;
    this->batteryCapacity = 0;
    this->batteryTargetCapacity = 0;
    this->intervals_staying = 0;
    this->arrival_interval = 0;
    this->name = "";

}
ElectricalVehicle::~ElectricalVehicle(){
    if(chargingplan_was_allocated){
        this->freeChargingPlan();
    }
}

/**
 * Allocates the memory for the solution of the chargingplan.
 * The Vehicle keeps ownership of this memory, it is freed in the destructor of the object.
 *
 * @param length
 */
void ElectricalVehicle::allocateChargingPlan(size_t length){
    this->chargingplan = new double[length];
    this->chargingplan_was_allocated = true;
}

void ElectricalVehicle::freeChargingPlan(){
    delete[] this->chargingplan;
}

void ElectricalVehicle::log(){
    printf("EV {");
    printf("minimumChargingPower: %d\t", minimumChargingPower);
    printf("maximumChargingPower: %d\t", maximumChargingPower);
    printf("batteryContent: %d\t", batteryContent);
    printf("batteryCapacity: %lu\t", batteryCapacity);
    printf("batteryTargetCapacity: %d\t", batteryTargetCapacity);
    printf("intervalsStaying: %d\t", (int) intervals_staying);
    printf("arrivalInterval %zu\t", arrival_interval);
    printf("name: %s", name);
    printf("}\n");
}
