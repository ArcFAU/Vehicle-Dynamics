classdef TestDualTrackSlipLapLevel21c < matlab.unittest.TestCase
    properties
        vehicle
        map
    end

    methods (TestClassSetup)
        function addMap(testCase)
            configuredVehicle = FAUmaxTauV2();
            generatedMap = lts.model.dualTrackSlip.buildPerformanceMap(...
                configuredVehicle, ...
                speedGrid_mps=[10, 20], lateralFractions=[0, 0.5, 1], ...
                boundaryIterations=6, showProgress=false);
            testCase.vehicle = configuredVehicle;
            testCase.map = generatedMap;
        end
    end

    methods (Test)
        function mapContainsSignedTurnBoundaries(testCase)
            performanceMap = testCase.map;
            testCase.verifySize(performanceMap.maximumLateralAcceleration_mps2, ...
                [2, 2]);
            testCase.verifySize(performanceMap.maximumAcceleration_mps2, ...
                [2, 3, 2]);
            testCase.verifyGreaterThan(min(...
                performanceMap.maximumLateralAcceleration_mps2, [], 'all'), 0);
            testCase.verifyEqual(...
                performanceMap.meta.interpolationSafetyFactor, 1.0);
        end

        function interpolatedLimitsUseLapInterface(testCase)
            model = lts.model.dualTrackSlip.create(testCase.vehicle, ...
                testCase.map);
            limits = model.accelerationLimits(15, 0);
            testCase.verifyTrue(limits.feasible);
            testCase.verifyGreaterThan(limits.maximumAcceleration_mps2, 0);
            testCase.verifyGreaterThan(limits.maximumDeceleration_mps2, 0);
        end

        function smallClosedLapConverges(testCase)
            model = lts.model.dualTrackSlip.create(testCase.vehicle, ...
                testCase.map);
            track.distance_m = (0:10:40).';
            track.curvature_1pm = [0; 0.02; -0.02; 0.01; 0];
            track.isClosed = true;
            track.meta.name = 'Level 2.1c test loop';
            result = lts.solver.solveLap(track, model, maximumSpeed_mps=20);
            testCase.verifyTrue(result.meta.converged);
            testCase.verifyGreaterThan(result.lapTime_s, 0);
            testCase.verifyEqual(result.meta.modelLevel, '2.1c');
        end

        function mismatchedRollDistributionIsRejected(testCase)
            modifiedVehicle = lts.suspension.applyRollStiffness(testCase.vehicle);
            testCase.verifyError(@() lts.model.dualTrackSlip.create(...
                modifiedVehicle, testCase.map), 'lts:model:MapConfiguration');
        end

        function mismatchedChassisStiffnessIsRejected(testCase)
            modifiedVehicle = testCase.vehicle;
            modifiedVehicle.chassis.torsionalStiffness_NmPerRad = ...
                1000 / deg2rad(1);
            testCase.verifyError(@() lts.model.dualTrackSlip.create(...
                modifiedVehicle, testCase.map), 'lts:model:MapConfiguration');
        end

        function anyVehicleDefinitionChangeIsRejected(testCase)
            modifiedVehicle = testCase.vehicle;
            modifiedVehicle.mass.total_kg = ...
                modifiedVehicle.mass.total_kg + 1;
            testCase.verifyError(@() lts.model.dualTrackSlip.create( ...
                modifiedVehicle, testCase.map), ...
                'lts:model:MapConfiguration');
        end
    end
end
