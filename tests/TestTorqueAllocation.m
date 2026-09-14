classdef TestTorqueAllocation < matlab.unittest.TestCase
    methods (Test)
        function defaultModeReproducesFixedDistribution(testCase)
            vehicle = FAUmaxTauV2();
            allocation = lts.control.allocateLongitudinalForces(vehicle, ...
                20, 1000 * ones(1, 4), zeros(1, 4), 800, "drive");
            testCase.verifyEqual(allocation.mode, 'fixed');
            testCase.verifyEqual(allocation.forces_N, 200 * ones(1, 4), ...
                'AbsTol', 1e-12);
            testCase.verifyTrue(allocation.feasible);
        end

        function idealFourMotorAllocationUsesUnequalWheelCapacity(testCase)
            vehicle = lts.control.applyIdealAllocation( ...
                FAUmaxTauV2("fourMotorAWD"));
            capacities_N = [100, 400, 400, 100];
            fixedVehicle = FAUmaxTauV2("fourMotorAWD");
            fixed = lts.control.allocateLongitudinalForces(fixedVehicle, ...
                20, capacities_N, zeros(1, 4), 800, "drive");
            ideal = lts.control.allocateLongitudinalForces(vehicle, ...
                20, capacities_N, zeros(1, 4), 800, "drive");
            testCase.verifyFalse(fixed.feasible);
            testCase.verifyTrue(ideal.feasible);
            testCase.verifyEqual(sum(ideal.forces_N), 800, ...
                'AbsTol', 1e-6);
            testCase.verifyEqual(ideal.residuals.yawMoment_Nm, 0, ...
                'AbsTol', 1e-6);
        end

        function sharedActuatorEnforcesEqualWheelForces(testCase)
            vehicle = lts.control.applyIdealAllocation( ...
                FAUmaxTauV2("dualMotorAWD"));
            allocation = lts.control.allocateLongitudinalForces(vehicle, ...
                10, 1000 * ones(1, 4), zeros(1, 4), 600, "drive");
            testCase.verifyTrue(allocation.feasible);
            testCase.verifyEqual(allocation.forces_N(1), ...
                allocation.forces_N(2), 'AbsTol', 1e-8);
            testCase.verifyEqual(allocation.forces_N(3), ...
                allocation.forces_N(4), 'AbsTol', 1e-8);
        end

        function idealRegenBypassesFixedControllerUtilization(testCase)
            fixed = FAUmaxTauV2();
            fixed.energy.initialSoc = 0.5;
            fixed.controls.regenAllocation.fixedUtilizationFactor = 0.5;
            ideal = lts.control.applyIdealAllocation(fixed);
            fixedStep = lts.energy.propagateSegment(fixed, 0.5, ...
                20, -5000, 10, 0.5);
            idealStep = lts.energy.propagateSegment(ideal, 0.5, ...
                20, -5000, 10, 0.5);
            testCase.verifyGreaterThan(idealStep.recoveredBattery_J, ...
                fixedStep.recoveredBattery_J);
            testCase.verifyEqual(fixedStep.regenAllocationFactor, 0.5);
            testCase.verifyEqual(idealStep.regenAllocationFactor, 1);
        end

        function idealEnvelopeClosesForceAndYawBalance(testCase)
            vehicle = lts.control.applyIdealAllocation( ...
                FAUmaxTauV2("fourMotorAWD"));
            point = lts.model.dualTrackEnvelope.operatingPoint(vehicle, ...
                18, 1.5, 7);
            testCase.verifyTrue(point.feasible);
            testCase.verifyEqual(struct2array(point.residuals), ...
                zeros(1, 3), 'AbsTol', 1e-6);
        end

        function fastIdealBoundaryMatchesDiagnosticAllocation(testCase)
            vehicle = lts.control.applyIdealAllocation( ...
                FAUmaxTauV2("fourMotorAWD"));
            speed_mps = [8; 15; 22; 28];
            ax_mps2 = [3; -7; 2; -4];
            ay_mps2 = [0; 5; -9; 11];
            state = lts.model.dualTrackEnvelope.calculateState(vehicle, ...
                speed_mps, ax_mps2, ay_mps2);
            fast = lts.model.dualTrackEnvelope.feasibleMask(vehicle, ...
                speed_mps, ax_mps2, ay_mps2);
            testCase.verifyEqual(fast, state.feasible);
        end
    end
end
