//
//  tests.cpp
//  libchargingplanoptimiser
//
//  Created by Eric Middelhove on 27.06.23.
//
#include "library.h"

#include <glpk.h>

#include <vector>
#include <stdlib.h>

#include "gtest/gtest.h"
#include "library.cpp"

using namespace std;

namespace {
    vector<ElectricalVehicle *> EVs = vector<ElectricalVehicle *>();

    glp_prob *problem;
    vector<int> ia;
    vector<int> ja;
    vector<double> ar;

    const size_t arrival_interval = 0;
    const size_t optimisation_frame_size = 96; // Formerly amount_of_15_minutes
    size_t constraintCount;
    size_t AMOUNT_COLS;

    class OptimizerTest : public ::testing::Test {

    protected:
        OptimizerTest * pFoo_;

        OptimizerTest() {
        }

        virtual ~OptimizerTest() {
        }

        virtual void SetUp() {

            for (int i = 0; i < 500; i++) {
                ElectricalVehicle *ev = new ElectricalVehicle(5, 27, 93.7, 0, 100, 8, arrival_interval,
                                                              "Taycan"); // Porsche Taycan Turbo S
                EVs.push_back(ev);
            }

            problem = glp_create_prob();
            glp_set_prob_name(problem, "unit test problem");
            glp_set_obj_dir(problem, GLP_MIN);

            ia = vector<int>();
            ja = vector<int>();
            ar = vector<double>();

            constraintCount = 0;

        }

        virtual void TearDown() {

            for (size_t i = 0; i < EVs.size(); i++) {
                delete EVs.at(i);
            }

            EVs = vector<ElectricalVehicle *>();

            ia = vector<int>();
            ja = vector<int>();
            ar = vector<double>();

            constraintCount = 0;

            glp_delete_prob(problem);

        }

    };

    TEST_F(OptimizerTest, NameIntegrity) {
        //    printf("\nNameIntegrity");
        // Setup
        char name[64] = "Taycan";

        // Do
        ElectricalVehicle ev = ElectricalVehicle(5, 27, 93.7, 0, 100, 8, arrival_interval, name);

        // Check

        EXPECT_STREQ(name, ev.name);
    }

    TEST_F(OptimizerTest, GlobalMaximumPowerConsumptionConstraint) {
        // Setup


        const size_t AMOUNT_ROWS = optimisation_frame_size;

        const int MAX_BUILDING_CAPACITY = 100;
        int dynamic_consumption[optimisation_frame_size];
        double pvPower[optimisation_frame_size];
        int baseBuildingCapacity = 100;

        for (int i = 0; i < optimisation_frame_size; i++) {
            dynamic_consumption[i] = 30;
            pvPower[i] = 100;
        }

        dynamic_consumption[0] = 40;
        dynamic_consumption[1] = 40;


        // Do
        addGlobalMaximumPowerConsumptionConstraint(problem, &EVs, &ia, &ja, &ar, &constraintCount,
                                                   optimisation_frame_size, dynamic_consumption, pvPower,
                                                   baseBuildingCapacity);

        // Check
        vector<int> compare_ia = vector<int>();
        vector<int> compare_ja = vector<int>();
        vector<double> compare_ar = vector<double>();

        for (int i = 1; i <= optimisation_frame_size; i++) {
            for (int j = 0; j < EVs.size(); j++) {
                compare_ia.push_back(i);
                compare_ja.push_back(i + j * (int) optimisation_frame_size);
                compare_ar.push_back(1);
            }
        }

        EXPECT_EQ(optimisation_frame_size, constraintCount);

        for (size_t i = 0; i < AMOUNT_ROWS; i++) {
            EXPECT_EQ(compare_ia.at(i), ia.at(i));
            EXPECT_EQ(compare_ja.at(i), ja.at(i));
            EXPECT_EQ(compare_ar.at(i), ar.at(i));
        }

    }

    TEST_F(OptimizerTest, BatteryCapacityReachedConstraint) {
//    printf("\nBatterCapacityReachedConstraint");
// Setup


// Do
        addTargetBatteryCapacityReachedConstraint(problem, &EVs, &ia, &ja, &ar, &constraintCount, arrival_interval,
                                                  AMOUNT_COLS, 15);

// Check
        vector<int> compare_ia = vector<int>();
        vector<int> compare_ja = vector<int>();
        vector<double> compare_ar = vector<double>();

        for (int i = 0; i < EVs.size(); i++) {

            int upperBound = (int) arrival_interval * (i + 1);
            int lowerBound = (int) arrival_interval * i;

            for (int j = 1; j <= AMOUNT_COLS; j++) {
                if (j > lowerBound && j <= upperBound) {
                    compare_ia.push_back(i + 1);
                    compare_ja.push_back(j);
                    compare_ar.push_back(1);
                }
            }
        }

        for (size_t i = 0; i < ia.size(); i++) {
            EXPECT_EQ(compare_ia.at(i), ia.at(i));
            EXPECT_EQ(compare_ja.at(i), ja.at(i));
            EXPECT_EQ(compare_ar.at(i), ar.at(i));
        }
    }

    TEST_F(OptimizerTest, ReachSoCInTimeConstraint) {
        //    printf("\nReachSocInTimeConstraint");

        // Setup
        const size_t AMOUNT_ROWS = EVs.size() * AMOUNT_COLS;

        // Do
        addReachSOCinTimeConstraint(problem, &EVs, &ia, &ja, &ar, &constraintCount, arrival_interval, AMOUNT_COLS, 15);

        // Setup Check
        vector<int> compare_ia = vector<int>();
        vector<int> compare_ja = vector<int>();
        vector<double> compare_ar = vector<double>();

        for (int i = 0; i < EVs.size(); i++) {
            string varname = "socInTime";
            varname.append(to_string(i));

            int upperBound = getIntervalNumberOfLeaving(0, EVs.at(i)->intervals_staying, 15);

            // int upperBound = AMOUNT_OF_15_MINUTES * i + 4;  // 4 = 1h
            // int lowerBound = AMOUNT_OF_15_MINUTES * i;
            int lowerBound = 0;  // Make charging available from the beginning

            for (int j = 1; j <= AMOUNT_COLS; j++) {
                if (j > lowerBound && j <= upperBound) {
                    compare_ja.push_back(j);
                    compare_ia.push_back((int) i + 1);
                    compare_ar.push_back(1);
                }
            }
        }

        // Check
        for (size_t i = 0; i < ia.size(); i++) {
            EXPECT_EQ(compare_ia.at(i), ia.at(i));
            EXPECT_EQ(compare_ja.at(i), ja.at(i));
            EXPECT_EQ(compare_ar.at(i), ar.at(i));
        }
    }

    TEST_F(OptimizerTest, BuildModel) {
        // Setup
        //    printf("\nBuildModel");

        double coefficients[optimisation_frame_size];
        int dynamicConsumption[optimisation_frame_size];
        double pvPower[optimisation_frame_size];
        const int baseBuildingCapacity = 100;
        for (size_t i = 0; i < optimisation_frame_size; i++) {
            coefficients[i] = 1;
            dynamicConsumption[i] = 30;
            pvPower[i] = 100;
        }

        glp_prob *prob = glp_create_prob();

        // Do
        buildModel(prob, &EVs, optimisation_frame_size, dynamicConsumption, coefficients, 15, pvPower,
                   baseBuildingCapacity);

        // Check
        const size_t AMOUNT_COLS = optimisation_frame_size * EVs.size();
        const size_t AMOUNT_ROWS = (AMOUNT_COLS / EVs.size()) + EVs.size() + EVs.size();

        int rows = glp_get_num_rows(prob);
        int cols = glp_get_num_cols(prob);

        EXPECT_EQ(AMOUNT_ROWS, rows);
        EXPECT_EQ(AMOUNT_COLS, cols);
    }

    TEST_F(OptimizerTest, GenerateCoefficientArray) {
        //    printf("\nGenerateCoefficientArray");
        // Setup
        double coeffArray[optimisation_frame_size] = {};
        int dynamicConsumption[optimisation_frame_size];

        double buildingCapacity = 51;
        double pvPower[optimisation_frame_size];

        for (int i = 0; i < optimisation_frame_size; i++) {
            dynamicConsumption[i] = 1;
            pvPower[i] = 50;
        }

        vector<double> energyCosts(optimisation_frame_size, 1.0);
        // Do

        generateCoefficientArray(coeffArray, optimisation_frame_size, energyCosts, 50, pvPower, 50, buildingCapacity,
                                 dynamicConsumption);


        // Check
        for (int i = 0; i < optimisation_frame_size; i++) {
            EXPECT_TRUE(coeffArray[i] > 0);
        }
    }
}

namespace {
    class ExecutionTest : public ::testing::Test {
    protected:
        ExecutionTest * pExecutionTest_;

        ExecutionTest() {
        }

        virtual ~ExecutionTest() {
        }

        virtual void SetUp() {
            int array[optimisation_frame_size] = {40, 15, 20, 15, 20, 20, 15, 5, 15, 15, 20, 50, 15, 20, 15, 20, 15,
                                                      15, 15, 15, 20, 50, 15, 20, 15, 20, 15, 15, 15, 15, 20, 50, 15,
                                                      20, 15, 20, 15, 15, 15, 15, 15, 20, 50, 15, 20, 15, 20, 15, 15,
                                                      15, 15, 20, 50, 0, 0, 0, 0, 15, 15, 15, 15, 20, 50, 15, 20, 10,
                                                      20, 10, 10, 10, 10, 20, 50, 10, 20, 10, 20, 10, 10, 10, 10, 20,
                                                      50, 10, 20, 10, 20, 50, 10, 20, 10, 20, 10, 10, 10, 10};


                for (int i = 0; i < optimisation_frame_size; i++) {
                    power_array[i] = array[i];
                    cost_array[i] = array[i];
                }


                return;
        }

        virtual void TearDown() {



        }

        vector<double> pvPower = vector<double>();
        vector<double> costs = vector<double>();

        const int EVCount = 2;
        ElectricalVehicle EV_arr[2] = {};
        double power_array[optimisation_frame_size];
        double cost_array[optimisation_frame_size];
        const int dynamicConsumption[optimisation_frame_size] = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
                                                                 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
                                                                 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
                                                                 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
                                                                 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
                                                                 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
                                                                 10, 10, 10, 10, 10, 10};
        int baseMaxBuildingCapacity = 250000;

    };


    TEST_F(ExecutionTest, executeOptimised) {
        //    printf("\nexecuteOptimised");
        // Setup
        ElectricalVehicle EV_arr[] = {
                ElectricalVehicle(5, 27, 93, 0, 100, 32, arrival_interval, "Taycan"),
                ElectricalVehicle(5, 27, 93, 0, 100, 32, arrival_interval, "Taycan2")
        };

        // Do
        int result = optimise(EV_arr, EVCount, 50, 50, dynamicConsumption, optimisation_frame_size, true, power_array,
                              cost_array, 15, baseMaxBuildingCapacity);
        // Check

        EXPECT_EQ(0, result);
    }

    TEST_F(ExecutionTest, executeNotOptimised) {
        //    printf("\nexecuteNotOptimised");
        // Setup
        ElectricalVehicle EV_arr[] = {
                ElectricalVehicle(5, 27, 93, 0, 100, 32, arrival_interval, "Taycan"),
                ElectricalVehicle(5, 27, 93, 0, 100, 32, arrival_interval, "Taycan2")
        };

        // Do
        int result = optimise(EV_arr, EVCount, 50, 50, dynamicConsumption, optimisation_frame_size, false, power_array,
                              cost_array, 15, baseMaxBuildingCapacity);
        // Check
        EXPECT_EQ(0, result);
    }

    TEST_F(ExecutionTest, Return10OnNoSolution) {
        //    printf("\nRetunr10OnNoSolution");
        // Setup
        ElectricalVehicle EV_arr[] = {
                ElectricalVehicle(5, 27, 93, 0, 100, 8, arrival_interval, "Taycan"),
                ElectricalVehicle(5, 27, 93, 0, 100, 8, arrival_interval, "Taycan2")
        };

        // Do
        int result = optimise(EV_arr, EVCount, 50, 50, dynamicConsumption, optimisation_frame_size, true, power_array,
                              cost_array, 15, 10);

        // Check
        EXPECT_EQ(10, result);
    }

    TEST_F(ExecutionTest, DoesNotChargeFullVehicles) {
        //    printf("\nDoNotChargoFullVehicles");

        const size_t length = 4;
        const int arrival_interval = 96;
        ElectricalVehicle arr[length] = {
                ElectricalVehicle(5, 100, 300, 100, 100, 8, arrival_interval, "Full Car 0"),
                ElectricalVehicle(5, 80, 300, 80, 80, 8, arrival_interval, "Full Car 1")
        };

        const int power_arr[] = {40, 15, 20, 15, 20, 20, 15, 5, 15, 15, 20, 50, 15, 20, 15, 20, 15, 15, 15, 15, 20, 50,
                                 15, 20, 15, 20, 15, 15, 15, 15, 20, 50, 15, 20, 15, 20, 15, 15, 15, 15, 15, 20, 50, 15,
                                 20, 15, 20, 15, 15, 15, 15, 20, 50, 20, 20, 20, 20, 15, 15, 15, 15, 20, 50, 15, 20, 10,
                                 20, 10, 10, 10, 10, 20, 50, 10, 20, 10, 20, 10, 10, 10, 10, 20, 50, 10, 20, 10, 20, 50,
                                 10, 20, 10, 20, 10, 10, 10, 10, 40, 15, 20, 15, 20, 20, 15, 5, 15, 15, 20, 50, 15, 20,
                                 15, 20, 15, 15, 15, 15, 20, 50, 15, 20, 15, 20, 15, 15, 15, 15, 20, 50, 15, 20, 15, 20,
                                 15, 15, 15, 15, 15, 20, 50, 15, 20, 15, 20, 15, 15, 15, 15, 20, 50, 1, 1, 1, 1, 15, 15,
                                 15, 15, 20, 50, 15, 20, 10, 20, 10, 10, 10, 10, 20, 50, 10, 20, 10, 20, 10, 10, 10, 10,
                                 20, 50, 10, 20, 10, 20, 50, 10, 20, 10, 20, 10, 10, 10, 10};

        const int cost_arr[] = {40, 15, 20, 15, 20, 20, 15, 5, 15, 15, 20, 50, 15, 20, 15, 20, 15, 15, 15, 15, 20, 50,
                                15, 20, 15, 20, 15, 15, 15, 15, 20, 50, 15, 20, 15, 20, 15, 15, 15, 15, 15, 20, 50, 15,
                                20, 15, 20, 15, 15, 15, 15, 20, 50, 20, 20, 20, 20, 15, 15, 15, 15, 20, 50, 15, 20, 10,
                                20, 10, 10, 10, 10, 20, 50, 10, 20, 10, 20, 10, 10, 10, 10, 20, 50, 10, 20, 10, 20, 50,
                                10, 20, 10, 20, 10, 10, 10, 10, 40, 15, 20, 15, 20, 20, 15, 5, 15, 15, 20, 50, 15, 20,
                                15, 20, 15, 15, 15, 15, 20, 50, 15, 20, 15, 20, 15, 15, 15, 15, 20, 50, 15, 20, 15, 20,
                                15, 15, 15, 15, 15, 20, 50, 15, 20, 15, 20, 15, 15, 15, 15, 20, 50, 1, 1, 1, 1, 15, 15,
                                15, 15, 20, 50, 15, 20, 10, 20, 10, 10, 10, 10, 20, 50, 10, 20, 10, 20, 10, 10, 10, 10,
                                20, 50, 10, 20, 10, 20, 50, 10, 20, 10, 20, 10, 10, 10, 10};

        int result = optimise(arr, length, 50, 50, dynamicConsumption, optimisation_frame_size, true, power_array,
                              cost_array, 15, baseMaxBuildingCapacity);

        EXPECT_EQ(0, result);

        for (int i = 0; i < length; i++) {
            for (int j = 0; j < arrival_interval; j++) {
                int power = arr[i].chargingplan[j];
                EXPECT_EQ(0, power);
            }
        }
    }
}