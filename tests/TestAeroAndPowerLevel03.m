classdef TestAeroAndPowerLevel03 < matlab.unittest.TestCase
    methods (Test)
        function aeroForcesScaleWithSpeedSquared(testCase)
            vehicle = FAUmaxTauV2();
            low = lts.aerodynamics.forces(vehicle.aerodynamics, 10);
            high = lts.aerodynamics.forces(vehicle.aerodynamics, 20);

            testCase.verifyEqual(high.drag_N, 4 * low.drag_N, 'AbsTol', 1e-12);
            testCase.verifyEqual(high.totalDownforce_N, ...
                4 * low.totalDownforce_N, 'AbsTol', 1e-12);
        end

        function driveForceRespectsMotorSpeed(testCase)
            vehicle = FAUmaxTauV2();
            atLimit = lts.powertrain.maximumDriveForce(vehicle.powertrain, ...
                vehicle.powertrain.maximumMotorSpeed_rpm * 2 * pi / 60 ...
                / vehicle.powertrain.gearRatio ...
                * vehicle.powertrain.effectiveTireRadius_m);
            overLimit = lts.powertrain.maximumDriveForce(vehicle.powertrain, ...
                1.01 * atLimit.maximumVehicleSpeed_mps);

            testCase.verifyGreaterThan(atLimit.maximumForce_N, 0);
            testCase.verifyEqual(overLimit.maximumForce_N, 0);
        end

        function pointMassAccelerationReducesWithDrag(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.model.pointMassAero.create(vehicle);
            slow = model.accelerationLimits(5, 0);
            fast = model.accelerationLimits(25, 0);

            testCase.verifyLessThan(fast.maximumAcceleration_mps2, ...
                slow.maximumAcceleration_mps2);
        end

        function zeroSpeedAeroForcesAreZero(testCase)
            vehicle = FAUmaxTauV2();
            aero = lts.aerodynamics.forces(vehicle.aerodynamics, 0);
            testCase.verifyEqual(struct2array(aero), zeros(1, 4));
        end

        function baselineAeroInputsAreCoupled(testCase)
            vehicle = FAUmaxTauV2();
            aero = vehicle.aerodynamics;
            testCase.verifyEqual(aero.liftCoefficientFront ...
                + aero.liftCoefficientRear, aero.totalLiftCoefficient, ...
                'AbsTol', 1e-12);
            testCase.verifyEqual(aero.liftCoefficientFront ...
                / aero.totalLiftCoefficient, ...
                aero.frontDownforceFraction, 'AbsTol', 1e-12);
        end

        function excessiveLateralDemandIsInfeasibleEvenWithoutDrag(testCase)
            vehicle = FAUmaxTauV2();
            vehicle.aerodynamics.dragCoefficient = 0;
            vehicle.aerodynamics.liftCoefficientFront = 0;
            vehicle.aerodynamics.liftCoefficientRear = 0;
            model = lts.model.pointMassAero.create(vehicle);

            limits = model.accelerationLimits(20, 100);

            testCase.verifyFalse(limits.feasible);
            maximumCornerSpeed_mps = ...
                model.maximumSpeedForCurvature(0.1);
            testCase.verifyLessThan(maximumCornerSpeed_mps, 20);
        end
    end
end
