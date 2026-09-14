classdef TestLoadSensitivePointMassLevel02 < matlab.unittest.TestCase
    methods (Test)
        function zeroLoadProducesZeroForce(testCase)
            vehicle = FAUmaxTauV2();
            force_N = lts.tire.maximumForce(vehicle.tireEnvelope, 0);
            testCase.verifyEqual(force_N, 0);
        end

        function zeroSensitivityRecoversMuTimesGravity(testCase)
            vehicle = FAUmaxTauV2();
            vehicle.tireEnvelope.loadSensitivityExponent = 0;
            model = lts.model.pointMassLoadSensitive.create(vehicle);
            limits = model.accelerationLimits(10, 0);
            expected_mps2 = ...
                vehicle.tireEnvelope.frictionCoefficientAtReferenceLoad * 9.81;
            testCase.verifyEqual(limits.maximumAcceleration_mps2, ...
                expected_mps2, 'AbsTol', 1e-12);
        end

        function addedMassReducesAccelerationWithLoadSensitivity(testCase)
            lightVehicle = FAUmaxTauV2();
            heavyVehicle = lightVehicle;
            heavyVehicle.mass.total_kg = 1.5 * lightVehicle.mass.total_kg;
            lightModel = lts.model.pointMassLoadSensitive.create(lightVehicle);
            heavyModel = lts.model.pointMassLoadSensitive.create(heavyVehicle);

            lightLimits = lightModel.accelerationLimits(10, 0);
            heavyLimits = heavyModel.accelerationLimits(10, 0);

            testCase.verifyLessThan(heavyLimits.maximumAcceleration_mps2, ...
                lightLimits.maximumAcceleration_mps2);
        end

        function combinedEnvelopeReachesZeroAtLateralLimit(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.model.pointMassLoadSensitive.create(vehicle);
            curvature_1pm = 0.05;
            speed_mps = model.maximumSpeedForCurvature(curvature_1pm);
            lateralAcceleration_mps2 = speed_mps^2 * curvature_1pm;

            limits = model.accelerationLimits(speed_mps, lateralAcceleration_mps2);

            testCase.verifyEqual(limits.maximumAcceleration_mps2, 0, ...
                'AbsTol', 1e-7);
        end
    end
end
