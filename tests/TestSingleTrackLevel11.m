classdef TestSingleTrackLevel11 < matlab.unittest.TestCase
    methods (Test)
        function staticLoadsMatchMassDistribution(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.model.singleTrack.create(vehicle);
            point = model.operatingPoint(0, 0);
            expectedLoads_N = vehicle.mass.total_kg * 9.81 ...
                * [vehicle.geometry.frontMassFraction, ...
                1 - vehicle.geometry.frontMassFraction];

            testCase.verifyEqual(point.axleNormalLoads_N, expectedLoads_N, ...
                'AbsTol', 1e-10);
        end

        function equilibriumResidualsClose(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.model.singleTrack.create(vehicle);
            point = model.operatingPoint(2.0, 6.0);

            residuals = struct2array(point.residuals);
            testCase.verifyEqual(residuals, zeros(size(residuals)), ...
                'AbsTol', 1e-10);
        end

        function accelerationTransfersLoadRearward(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.model.singleTrack.create(vehicle);
            staticPoint = model.operatingPoint(0, 0);
            acceleratingPoint = model.operatingPoint(2, 0);

            testCase.verifyLessThan(acceleratingPoint.axleNormalLoads_N(1), ...
                staticPoint.axleNormalLoads_N(1));
            testCase.verifyGreaterThan(acceleratingPoint.axleNormalLoads_N(2), ...
                staticPoint.axleNormalLoads_N(2));
        end

        function leftAndRightLimitsAreSymmetric(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.model.singleTrack.create(vehicle);
            left = model.accelerationLimits(15, 5);
            right = model.accelerationLimits(15, -5);

            testCase.verifyEqual(left.maximumAcceleration_mps2, ...
                right.maximumAcceleration_mps2, 'AbsTol', 1e-10);
            testCase.verifyEqual(left.maximumDeceleration_mps2, ...
                right.maximumDeceleration_mps2, 'AbsTol', 1e-10);
        end

        function commonLapSolverAcceptsSingleTrack(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.model.singleTrack.create(vehicle);
            track.meta.name = 'Single-track interface test';
            track.distance_m = linspace(0, 100, 51)';
            track.curvature_1pm = 0.03 * ones(51, 1);
            track.isClosed = true;

            result = lts.solver.solveLap(track, model);

            testCase.verifyTrue(result.meta.converged);
            testCase.verifyEqual(result.speed_mps(1), result.speed_mps(end), ...
                'AbsTol', 1e-12);
        end
    end
end
