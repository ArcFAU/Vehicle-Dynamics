classdef TestDualTrackEnvelopeLevel20 < matlab.unittest.TestCase
    methods (Test)
        function fourWheelLoadsSumToTotalVerticalLoad(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.model.dualTrackEnvelope.create(vehicle);
            point = model.operatingPoint(20, 1, 6);
            expected_N = vehicle.mass.total_kg * 9.81 ...
                + point.aero.totalDownforce_N;

            testCase.verifyEqual(sum(point.normalLoads_N), expected_N, ...
                'AbsTol', 1e-10);
        end

        function positiveLateralAccelerationLoadsRightWheels(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.model.dualTrackEnvelope.create(vehicle);
            point = model.operatingPoint(15, 0, 5);

            testCase.verifyGreaterThan(point.normalLoads_N(2), point.normalLoads_N(1));
            testCase.verifyGreaterThan(point.normalLoads_N(4), point.normalLoads_N(3));
        end

        function leftAndRightStatesMirror(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.model.dualTrackEnvelope.create(vehicle);
            left = model.operatingPoint(15, 0, 5);
            right = model.operatingPoint(15, 0, -5);

            testCase.verifyEqual(left.normalLoads_N, ...
                right.normalLoads_N([2, 1, 4, 3]), 'AbsTol', 1e-10);
            testCase.verifyEqual(left.tireUtilization, ...
                right.tireUtilization([2, 1, 4, 3]), 'AbsTol', 1e-10);
        end

        function operatingPointClosesEquilibria(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.model.dualTrackEnvelope.create(vehicle);
            point = model.operatingPoint(15, 1, 5);

            testCase.verifyEqual(struct2array(point.residuals), zeros(1, 3), ...
                'AbsTol', 1e-10);
        end

        function zeroCgHeightRemovesLateralTransfer(testCase)
            vehicle = FAUmaxTauV2();
            vehicle.geometry.cgHeight_m = eps;
            model = lts.model.dualTrackEnvelope.create(vehicle);
            point = model.operatingPoint(15, 0, 5);

            testCase.verifyEqual(point.normalLoads_N(1), point.normalLoads_N(2), ...
                'AbsTol', 1e-10);
            testCase.verifyEqual(point.normalLoads_N(3), point.normalLoads_N(4), ...
                'AbsTol', 1e-10);
        end

        function optimizedFeasibilityMatchesDiagnosticState(testCase)
            vehicle = FAUmaxTauV2();
            speed_mps = [5; 15; 25; 32];
            ax_mps2 = [2; -5; 1; 0];
            ay_mps2 = [0; 4; -8; 12];
            state = lts.model.dualTrackEnvelope.calculateState(vehicle, ...
                speed_mps, ax_mps2, ay_mps2);
            optimized = lts.model.dualTrackEnvelope.feasibleMask(vehicle, ...
                speed_mps, ax_mps2, ay_mps2);

            testCase.verifyEqual(optimized, state.feasible);
        end
    end
end
