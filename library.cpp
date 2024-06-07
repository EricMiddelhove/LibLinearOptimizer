//
//  optimiser.cpp
//  MHP-LibLinearOptimizer
//
//  Created by Eric Middelhove on 27.06.23.
//

// TODO Optimisation: Inform Algorithm Of in Future arriving vehicles (Adjusting all Constraints, Adjusting vehicle Class)

#include "library.h"

#include <glpk.h>
#include <stdio.h>

#include <string>
#include <vector>

using namespace std;

// Helpers
int calculateTargetBatteryCapacity(unsigned long batteryCapacity, int targetSoC) {
    return batteryCapacity * targetSoC / 100;
}

/*
* This function assumes that the car arrives and is ready to charge at interval 0
*/
int getIntervalNumberOfLeaving(const int arrivalInterval, int intervals_staying, const int intervalDuration) {
    return arrivalInterval + intervals_staying;
}


int getCurrentBuildingCapacity(const double pvPower[], const int dynamicConsumption[], const int baseBuildingCapacity, int interval){
    int i = interval;

    int result = (int) baseBuildingCapacity - (int) dynamicConsumption[i] + pvPower[i];

    return result;
}

void generateCoefficientArray(double coeffArray[], size_t arraySize, vector<double> energyCosts, double costWeight, const double pvPower[], double emmissionWeight, double buildingCapacity, const int dynamicConsumption[]) {
    for (int i = 0; i < arraySize; i++) {
        // TODO: emmisionMix needs to be calculated

        float highestPrice = 10;

        double currentBuildingCap = getCurrentBuildingCapacity(pvPower, dynamicConsumption, buildingCapacity, i);

        float pvPercent = pvPower[i] * 100 / currentBuildingCap; // How much of the current building cap is available
        float pricePercent = energyCosts[i] * 100 / highestPrice;

        double coefficient = (costWeight * pricePercent + (100 - pvPercent) * emmissionWeight);

        *(coeffArray + i) = coefficient;
    }
}

// Constraints
void addPowerForEachVehicleConstraint(glp_prob* problem, vector<ElectricalVehicle*>* EVs, vector<int>* ia, vector<int>* ja, vector<double>* ar, size_t* constraintCount, size_t AMOUNT_COLS, size_t AMOUNT_OF_15_MINUTES) {

    for (int i = 0; i < EVs->size(); i++){

        for(int j = 0; j < AMOUNT_OF_15_MINUTES; j++){
            int index = i * AMOUNT_OF_15_MINUTES + j + 1;

            string varname = "ev";
            varname.append(to_string(i));
            varname.append("+");
            varname.append(to_string(j));

            // EVs->at(i)->log();

            glp_set_col_name(problem, index, varname.c_str());
            if ((double) EVs->at(i)->maximumChargingPower == 0.0 ) {
                glp_set_col_bnds(problem, index, GLP_FX, 0, 0);
            }else{
                glp_set_col_bnds(problem, index, GLP_DB, 0, (double) EVs->at(i)->maximumChargingPower);
            }
        }

    }

    return;
}

void addGlobalMaximumPowerConsumptionConstraint(glp_prob* problem, vector<ElectricalVehicle*>* EVs, vector<int>* ia, vector<int>* ja, vector<double>* ar, size_t* constraintCounter, size_t optimisationFrameSize, const int dynamicConsumption[], const double pvPower[], const int baseBuildingCapacity) {
    size_t evSize = EVs->size();

    glp_add_rows(problem, optimisationFrameSize);

    // Maximum power consumption constraint -> Sum of all charging-processes must not exceed maximum building capacity per timeframe

    for (int j = 1; j <= optimisationFrameSize; j++) {
        string varname = "gloMaxPowCons";
        varname.append(to_string(j));

        *constraintCounter = *constraintCounter + 1;
        glp_set_row_name(problem, (int) *constraintCounter, varname.c_str());

        int currentBuildingCapacity = getCurrentBuildingCapacity(pvPower, dynamicConsumption, baseBuildingCapacity, j - 1);

//    printf("%d\n", currentBuildingCapacity);

        glp_set_row_bnds(problem, (int) *constraintCounter, GLP_UP, currentBuildingCapacity, currentBuildingCapacity);

        for (int i = 0; i < evSize; i++) {
            ia->push_back((int) *constraintCounter);
            ja->push_back(j + i * (int)optimisationFrameSize);
            ar->push_back(1.0);
        }
    }
}

void addTargetBatteryCapacityReachedConstraint(glp_prob* problem, vector<ElectricalVehicle*>* EVs, vector<int>* ia, vector<int>* ja, vector<double>* ar, size_t* constraintCounter, size_t AMOUNT_OF_15_MINUTES, size_t AMOUNT_COLS, const int intervalDuration) {
    size_t evSize = EVs->size();

    glp_add_rows(problem, evSize);

    for (int i = 0; i < evSize; i++) {
        string varname = "targCapReached";
        varname.append(to_string(i));

        *constraintCounter = *constraintCounter + 1;
        glp_set_row_name(problem, (int) *constraintCounter, varname.c_str());

        unsigned long remainingKWinBattery = EVs->at(i)->batteryContent;

        unsigned long targetCapacity = EVs->at(i)->batteryTargetCapacity;

        double intervals_in_one_hour = 60 / intervalDuration;

        int chargingDiff = targetCapacity - remainingKWinBattery;
        if(chargingDiff <= 0){
            chargingDiff = 0;
            glp_set_row_bnds(problem, (int) *constraintCounter, GLP_FX, 0.0, 0.0);  // Target SoC 100%

        }else{
            glp_set_row_bnds(problem, (int) *constraintCounter, GLP_LO, chargingDiff * intervals_in_one_hour, 0.0);  // Target SoC 100%
        }


        int upperBound = (int) AMOUNT_OF_15_MINUTES * (i + 1);
        int lowerBound = (int) AMOUNT_OF_15_MINUTES * i;

        for (int j = 1; j <= AMOUNT_COLS; j++) {
            if (j > lowerBound && j <= upperBound) {
                ja->push_back(j);
                ia->push_back((int) *constraintCounter);
                ar->push_back(1);
            }
        }
    }
}

void addReachSOCinTimeConstraint(glp_prob* problem, vector<ElectricalVehicle*>* EVs, vector<int>* ia, vector<int>* ja, vector<double>* ar, size_t* constraintCounter, size_t AMOUNT_OF_15_MINUTES, size_t AMOUNT_COLS, const int intervalDuration) {
    size_t evSize = EVs->size();

    for (int i = 0; i < evSize; i++) {

        if (EVs->at(i)->intervals_staying == 0){
            continue;
        }

        glp_add_rows(problem, 1);
        string varname = "socInTime";
        varname.append(to_string(i));

        *constraintCounter = *constraintCounter + 1;
        glp_set_row_name(problem, (int) *constraintCounter, varname.c_str());

        unsigned long remainingKWinBattery = EVs->at(i)->batteryContent;
        unsigned long targetCapacity = EVs->at(i)->batteryTargetCapacity;

        unsigned long chargingDiff = targetCapacity - remainingKWinBattery;

        double intervals_in_one_hour = 60 / intervalDuration;

        if(chargingDiff <= 0){
            chargingDiff = 0;
            glp_set_row_bnds(problem, (int) *constraintCounter, GLP_FX, 0.0, 0.0);
        }else{
            glp_set_row_bnds(problem, (int) *constraintCounter, GLP_LO, chargingDiff * intervals_in_one_hour, 0.0);
        }


        int lowerBound = EVs->at(i)->arrival_interval + i * AMOUNT_OF_15_MINUTES;

        int upperBound = getIntervalNumberOfLeaving(lowerBound, EVs->at(i)->intervals_staying, intervalDuration);

        for (int j = 1; j <= AMOUNT_COLS; j++) {
            if (j > lowerBound && j <= upperBound) {
                ja->push_back(j);
                ia->push_back((int) *constraintCounter);
                ar->push_back(1);
            }
        }
    }
}

// GLPK MODEL
void buildModel(glp_prob* problem, vector<ElectricalVehicle*>* EVs, const int optimisation_frame_size, const int dynamicConsumption[], double coefficients[], const int intervalDuration, const double pvPower[], const int baseBuildingCapacity) {
    const size_t evSize = EVs->size();

    const size_t AMOUNT_COLS = optimisation_frame_size * evSize;
    printf("evSize: %zu\n", evSize);
    // const size_t AMOUNT_ROWS = AMOUNT_COLS + (AMOUNT_COLS / evSize) + evSize + evSize;

    vector<int> ia;
    vector<int> ja;
    vector<double> ar;

    ia.push_back(0);
    ja.push_back(0);
    ar.push_back(0);

    glp_set_prob_name(problem, "optimizer model");
    glp_set_obj_dir(problem, GLP_MIN);

    // auto start = high_resolution_clock::now();

    printf("219 Cols %d\n", (int) AMOUNT_COLS);
    glp_add_cols(problem, (int) AMOUNT_COLS);  // Variables

    size_t amountOfConstraints = 0;

    // power for each vehicle constraint
    addPowerForEachVehicleConstraint(problem, EVs, &ia, &ja, &ar, &amountOfConstraints, AMOUNT_COLS, optimisation_frame_size);

    // Maximum power consumption constraint
    addGlobalMaximumPowerConsumptionConstraint(problem, EVs, &ia, &ja, &ar, &amountOfConstraints, optimisation_frame_size, dynamicConsumption, pvPower, baseBuildingCapacity);

    // Max Capacity is reached constraint
    addTargetBatteryCapacityReachedConstraint(problem, EVs, &ia, &ja, &ar, &amountOfConstraints, optimisation_frame_size, AMOUNT_COLS, intervalDuration);

    // Reach SoC in time constraint
    addReachSOCinTimeConstraint(problem, EVs, &ia, &ja, &ar, &amountOfConstraints, optimisation_frame_size, AMOUNT_COLS, intervalDuration);

    glp_load_matrix(problem, (int) ar.size() - 1, ia.data(), ja.data(), ar.data());

    for (int i = 0; i < AMOUNT_COLS; i++) {
        int coeff_index = (i % optimisation_frame_size);
        double coefficient = coefficients[coeff_index];
        glp_set_obj_coef(problem, i + 1, coefficient);
    }

}

void extractSolution(glp_prob* problem, double** solution, vector<ElectricalVehicle*>* EVs, const int amount_of_15_minutes, int base_max_building_capacity, const int dynamic_consumption[]){

    double sum_energy_consumption[amount_of_15_minutes];
    // solution length always = EVs->size();

    for (size_t i = 0; i < EVs->size(); i++) {
        for (int j = 1; j < amount_of_15_minutes + 1; j++) {
            double value = glp_get_col_prim(problem, j + (int) (i * amount_of_15_minutes));

            float minimumChargingPower = EVs->at(i)->minimumChargingPower;

            if (value > 0 && value <= minimumChargingPower) {
                if (sum_energy_consumption[j - 1] < base_max_building_capacity - dynamic_consumption[i] - minimumChargingPower) {
                    value = minimumChargingPower;
                }
            }
            solution[i][j - 1] = value;
            sum_energy_consumption[j - 1] += value;
        }
    }
}

char optimise(ElectricalVehicle EV_arr[], size_t evCount, unsigned short emissionWeight, unsigned short energyWeight, const int dynamicConsumption[], const unsigned int optimisationFrameSize, bool optimise,const double pvPowerRooftop_arr[], const double energyCosts_arr[], const int intervalDuration, const int baseMaxBuildingCapacity) {

    if (evCount == 0) { // GLPK cannot be called with an empty array or else it will run into an error
        return 0;
    }

    double coeffArray[optimisationFrameSize];

    vector<ElectricalVehicle*> EVs;

    for(size_t i = 0; i < evCount; i++){

//        (*(EV_arr + i)).log();
        EVs.push_back(EV_arr + i);
    }

    if (optimise) {
        vector<double> energyCosts;
        vector<double> pvPowerRooftop;

        for(size_t i = 0; i < optimisationFrameSize; i++){
            energyCosts.push_back(energyCosts_arr[i]);
            pvPowerRooftop.push_back(pvPowerRooftop_arr[i]);
        }

        generateCoefficientArray(coeffArray, optimisationFrameSize, energyCosts, energyWeight, pvPowerRooftop_arr, emissionWeight, baseMaxBuildingCapacity, dynamicConsumption);
    } else {
        for (int i = 0; i < optimisationFrameSize; i++) {
            coeffArray[i] = 1;
        }
    }


    glp_prob* problem = glp_create_prob(); // Alloc

    buildModel(problem, &EVs, optimisationFrameSize, dynamicConsumption, coeffArray, intervalDuration, pvPowerRooftop_arr, baseMaxBuildingCapacity);

    glp_smcp parameters;
    glp_init_smcp(&parameters);
    parameters.meth = GLP_DUALP;
    parameters.presolve = GLP_ON;
    parameters.msg_lev = GLP_MSG_ERR;

    glp_cpxcp write_params;

    int solutionStatus = glp_simplex(problem, &parameters);

    if (solutionStatus != 0){
        printf("No solution! \n");
        glp_write_lp(problem, &write_params, "problem.txt");

        glp_delete_prob(problem); // Free
        return 10;
    }

    double* solution[EVs.size()];
    for(size_t i = 0; i < EVs.size(); i++){
        solution[i] = new double[optimisationFrameSize];
    }

    extractSolution(problem, solution, &EVs, optimisationFrameSize, baseMaxBuildingCapacity, dynamicConsumption);
//    glp_print_sol(problem,"solution.txt");

    for(size_t i = 0; i < evCount; i++){
        EVs.at(i)->allocateChargingPlan(optimisationFrameSize);

        for(size_t j = 0; j < optimisationFrameSize; j++){
            double value = solution[i][j];
            EVs.at(i)->chargingplan[j] = value;
        }
    }


    glp_delete_prob(problem); // Free
    for (size_t i = 0; i < EVs.size(); i++) {
        delete[] solution[i];
    }

    return 0;
}

