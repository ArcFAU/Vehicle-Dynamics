classdef TestTireModelSelectionLevel25 < matlab.unittest.TestCase
    methods (Test)
        function distanceCurvatureMatrixdapterPreservesReferenceForcesExactly(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.tire.create(vehicle);
            fit = model.parameters;
            reference = lts.tire.combinedSlipForces(fit, ...
                650, deg2rad(-1), 0.08, deg2rad(4));
            selected = lts.tire.evaluate(model, ...
                650, deg2rad(-1), 0.08, deg2rad(4), 20);
            testCase.verifyEqual(selected.longitudinal_N, ...
                reference.longitudinal_N, 'AbsTol', 0);
            testCase.verifyEqual(selected.lateral_N, ...
                reference.lateral_N, 'AbsTol', 0);
        end

        function tmeasyMatchesAdtmReferenceOperatingPoint(testCase)
            vehicle = lts.tire.selectModel(FAUmaxTauV2(), ...
                "tmeasyAdtm");
            model = lts.tire.create(vehicle);
            forces = lts.tire.evaluate(model, 575, 0, 0.1, ...
                deg2rad(5), 20);
            testCase.verifyEqual(forces.longitudinal_N, ...
                1196.30974824491, 'AbsTol', 1e-8);
            testCase.verifyEqual(forces.lateral_N, ...
                714.753636368451, 'AbsTol', 1e-8);
            testCase.verifyEqual(forces.aligningMoment_Nm, ...
                -5.37755875809605, 'AbsTol', 1e-10);
            testCase.verifyEqual(forces.actualSlipRatio, ...
                0.099972765136475, ...
                'AbsTol', 1e-12);
        end

        function tmeasyForceSignsFollowNextGenConvention(testCase)
            vehicle = lts.tire.selectModel(FAUmaxTauV2(), ...
                "tmeasyAdtm");
            model = lts.tire.create(vehicle);
            positive = lts.tire.evaluate(model, 575, 0, 0.05, ...
                deg2rad(3), 15);
            negative = lts.tire.evaluate(model, 575, 0, -0.05, ...
                deg2rad(-3), 15);
            testCase.verifyGreaterThan(positive.longitudinal_N, 0);
            testCase.verifyGreaterThan(positive.lateral_N, 0);
            testCase.verifyLessThan(negative.longitudinal_N, 0);
            testCase.verifyLessThan(negative.lateral_N, 0);
        end

        function tmeasyDualTrackOperatingPointConverges(testCase)
            vehicle = lts.tire.selectModel(FAUmaxTauV2(), ...
                "tmeasyAdtm");
            point = lts.model.dualTrackSlip.operatingPoint(vehicle, ...
                18, 1.5, 7);
            testCase.verifyTrue(point.feasible);
            testCase.verifyEqual(point.tireModel, 'tmeasyAdtm');
            testCase.verifyEqual(point.solver.initializationEnvelope, ...
                'numericalInitializationOnly');
            testCase.verifyLessThan(norm(point.solver.normalizedResidual, Inf), ...
                1e-6);
        end

        function performanceMapRejectsDifferentTireModel(testCase)
            fittedVehicle = FAUmaxTauV2();
            map = lts.model.dualTrackSlip.buildPerformanceMap(fittedVehicle, ...
                speedGrid_mps=[10, 20], lateralFractions=[0, 1], ...
                boundaryIterations=2, showProgress=false);
            tmeasyVehicle = lts.tire.selectModel(fittedVehicle, ...
                "tmeasyAdtm");
            testCase.verifyError(@() lts.model.dualTrackSlip.create( ...
                tmeasyVehicle, map), 'lts:model:MapConfiguration');
        end

        function tmeasyPerformanceMapRunsThroughLapInterface(testCase)
            vehicle = lts.tire.selectModel(FAUmaxTauV2(), ...
                "tmeasyAdtm");
            map = lts.model.dualTrackSlip.buildPerformanceMap(vehicle, ...
                speedGrid_mps=[10, 20], lateralFractions=[0, 1], ...
                boundaryIterations=3, showProgress=false);
            model = lts.model.dualTrackSlip.create(vehicle, map);
            track.distance_m = (0:10:40).';
            track.curvature_1pm = [0; 0.02; -0.02; 0.01; 0];
            track.isClosed = true;
            track.meta.name = 'TMeasy interface test loop';
            result = lts.solver.solveLap(track, model, maximumSpeed_mps=20);
            testCase.verifyTrue(result.meta.converged);
            testCase.verifyEqual(result.meta.modelLevel, '2.5c');
            testCase.verifyGreaterThan(result.lapTime_s, 0);
            testCase.verifyEqual(map.meta.tireModel, 'tmeasyAdtm');
        end
    end
end
