add_test([=[OptimizerTest.NameIntegrity]=]  /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/lib_tests [==[--gtest_filter=OptimizerTest.NameIntegrity]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[OptimizerTest.NameIntegrity]=]  PROPERTIES WORKING_DIRECTORY /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
add_test([=[OptimizerTest.GlobalMaximumPowerConsumptionConstraint]=]  /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/lib_tests [==[--gtest_filter=OptimizerTest.GlobalMaximumPowerConsumptionConstraint]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[OptimizerTest.GlobalMaximumPowerConsumptionConstraint]=]  PROPERTIES WORKING_DIRECTORY /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
add_test([=[OptimizerTest.BatteryCapacityReachedConstraint]=]  /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/lib_tests [==[--gtest_filter=OptimizerTest.BatteryCapacityReachedConstraint]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[OptimizerTest.BatteryCapacityReachedConstraint]=]  PROPERTIES WORKING_DIRECTORY /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
add_test([=[OptimizerTest.ReachSoCInTimeConstraint]=]  /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/lib_tests [==[--gtest_filter=OptimizerTest.ReachSoCInTimeConstraint]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[OptimizerTest.ReachSoCInTimeConstraint]=]  PROPERTIES WORKING_DIRECTORY /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
add_test([=[OptimizerTest.BuildModel]=]  /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/lib_tests [==[--gtest_filter=OptimizerTest.BuildModel]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[OptimizerTest.BuildModel]=]  PROPERTIES WORKING_DIRECTORY /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
add_test([=[OptimizerTest.GenerateCoefficientArray]=]  /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/lib_tests [==[--gtest_filter=OptimizerTest.GenerateCoefficientArray]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[OptimizerTest.GenerateCoefficientArray]=]  PROPERTIES WORKING_DIRECTORY /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
add_test([=[ExecutionTest.executeOptimised]=]  /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/lib_tests [==[--gtest_filter=ExecutionTest.executeOptimised]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[ExecutionTest.executeOptimised]=]  PROPERTIES WORKING_DIRECTORY /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
add_test([=[ExecutionTest.executeNotOptimised]=]  /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/lib_tests [==[--gtest_filter=ExecutionTest.executeNotOptimised]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[ExecutionTest.executeNotOptimised]=]  PROPERTIES WORKING_DIRECTORY /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
add_test([=[ExecutionTest.Return10OnNoSolution]=]  /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/lib_tests [==[--gtest_filter=ExecutionTest.Return10OnNoSolution]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[ExecutionTest.Return10OnNoSolution]=]  PROPERTIES WORKING_DIRECTORY /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
add_test([=[ExecutionTest.DoesNotChargeFullVehicles]=]  /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/lib_tests [==[--gtest_filter=ExecutionTest.DoesNotChargeFullVehicles]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[ExecutionTest.DoesNotChargeFullVehicles]=]  PROPERTIES WORKING_DIRECTORY /Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
set(  lib_tests_TESTS OptimizerTest.NameIntegrity OptimizerTest.GlobalMaximumPowerConsumptionConstraint OptimizerTest.BatteryCapacityReachedConstraint OptimizerTest.ReachSoCInTimeConstraint OptimizerTest.BuildModel OptimizerTest.GenerateCoefficientArray ExecutionTest.executeOptimised ExecutionTest.executeNotOptimised ExecutionTest.Return10OnNoSolution ExecutionTest.DoesNotChargeFullVehicles)