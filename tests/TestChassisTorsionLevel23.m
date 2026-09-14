classdef TestChassisTorsionLevel23 < matlab.unittest.TestCase
    methods (Test)
        function rigidChassisRecoversElasticDistribution(testCase)
            vehicle = FAUmaxTauV2();
            roll = lts.suspension.rollProperties(vehicle);
            coupling = lts.chassis.rollCoupling(vehicle);
            testCase.verifyEqual(coupling.frontRollMomentFraction, ...
                roll.frontRollMomentFraction, 'AbsTol', 1e-12);
            testCase.verifyEqual(coupling.chassisTwistPerMoment_radPerNm, 0);
        end

        function uncoupledAxlesFollowAppliedMoments(testCase)
            vehicle = FAUmaxTauV2();
            vehicle.chassis.torsionalStiffness_NmPerRad = 0;
            coupling = lts.chassis.rollCoupling(vehicle);
            testCase.verifyEqual(coupling.frontRollMomentFraction, ...
                vehicle.geometry.frontMassFraction, 'AbsTol', 1e-12);
            testCase.verifyEqual(coupling.equilibriumResidual, 0, ...
                'AbsTol', 1e-12);
        end

        function finiteStiffnessLiesBetweenLimits(testCase)
            vehicle = FAUmaxTauV2();
            vehicle.chassis.torsionalStiffness_NmPerRad = 2000 / deg2rad(1);
            finite = lts.chassis.rollCoupling(vehicle);
            rigid = lts.suspension.rollProperties(vehicle);
            testCase.verifyGreaterThan(finite.frontRollMomentFraction, ...
                vehicle.geometry.frontMassFraction);
            testCase.verifyLessThan(finite.frontRollMomentFraction, ...
                rigid.frontRollMomentFraction);
            testCase.verifyGreaterThan(abs(finite.chassisTwistPerMoment_radPerNm), 0);
        end

        function applyingStiffnessLeavesOriginalUnchanged(testCase)
            vehicle = FAUmaxTauV2();
            stiffness_NmPerRad = 1000 / deg2rad(1);
            [configuredVehicle, coupling] = ...
                lts.chassis.applyTorsionalStiffness(vehicle, stiffness_NmPerRad);
            testCase.verifyEqual(vehicle.chassis.torsionalStiffness_NmPerRad, Inf);
            testCase.verifyEqual(...
                configuredVehicle.chassis.torsionalStiffness_NmPerRad, ...
                stiffness_NmPerRad);
            testCase.verifyEqual(...
                configuredVehicle.suspension.frontRollMomentFraction, ...
                coupling.frontRollMomentFraction);
        end
    end
end
