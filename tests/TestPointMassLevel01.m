classdef TestPointMassLevel01 < matlab.unittest.TestCase
    methods (Test)
        function constantRadiusMatchesAnalyticSpeed(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.model.pointMass.create(vehicle);
            radius_m = 20;
            curvature_1pm = ones(101, 1) / radius_m;
            expectedSpeed_mps = sqrt(...
                vehicle.tireApproximation.frictionCoefficient * 9.81 * radius_m);

            actualSpeed_mps = model.maximumSpeedForCurvature(curvature_1pm);

            testCase.verifyEqual(actualSpeed_mps, ...
                expectedSpeed_mps * ones(size(curvature_1pm)), ...
                'AbsTol', 1e-12);
        end

        function leftAndRightCornersAreSymmetric(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.model.pointMass.create(vehicle);
            curvature_1pm = [0.05; -0.05];

            speed_mps = model.maximumSpeedForCurvature(curvature_1pm);

            testCase.verifyEqual(speed_mps(1), speed_mps(2), 'AbsTol', 1e-12);
        end

        function straightAccelerationMatchesEnergyEquation(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.model.pointMass.create(vehicle);
            track = TestPointMassLevel01.straightTrack(50);
            expectedSpeed_mps = sqrt(2 ...
                * vehicle.tireApproximation.frictionCoefficient * 9.81 ...
                * track.distance_m(end));

            result = lts.solver.solveLap(track, model, ...
                initialSpeed_mps=0, finalSpeed_mps=100, maximumSpeed_mps=100);

            testCase.verifyEqual(result.speed_mps(end), expectedSpeed_mps, ...
                'AbsTol', 1e-10);
        end

        function straightBrakingMatchesEnergyEquation(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.model.pointMass.create(vehicle);
            track = TestPointMassLevel01.straightTrack(50);
            expectedStartSpeed_mps = sqrt(2 ...
                * vehicle.tireApproximation.frictionCoefficient * 9.81 ...
                * track.distance_m(end));

            result = lts.solver.solveLap(track, model, ...
                initialSpeed_mps=100, finalSpeed_mps=0, maximumSpeed_mps=100);

            testCase.verifyEqual(result.speed_mps(1), expectedStartSpeed_mps, ...
                'AbsTol', 1e-10);
        end
    end

    methods (Static, Access = private)
        function track = straightTrack(length_m)
            track.meta.name = 'Analytic straight';
            track.distance_m = linspace(0, length_m, 101)';
            track.curvature_1pm = zeros(101, 1);
            track.isClosed = false;
        end
    end
end
