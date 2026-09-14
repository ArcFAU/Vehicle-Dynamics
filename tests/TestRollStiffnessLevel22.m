classdef TestRollStiffnessLevel22 < matlab.unittest.TestCase
    methods (Test)
        function axleStiffnessUsesWheelRateAndTrack(testCase)
            vehicle = FAUmaxTauV2();
            properties = lts.suspension.rollProperties(vehicle);
            expectedFrontWheelRate_Npm = 158e3 / 1.219 ^ 2;
            expectedFrontRollStiffness_NmPerRad = ...
                0.5 * expectedFrontWheelRate_Npm * 1.240 ^ 2;
            testCase.verifyEqual(properties.frontWheelRate_Npm, ...
                expectedFrontWheelRate_Npm, 'RelTol', 1e-12);
            testCase.verifyEqual(properties.frontAxleRollStiffness_NmPerRad, ...
                expectedFrontRollStiffness_NmPerRad, 'RelTol', 1e-12);
        end

        function derivedDistributionIsFrontBiased(testCase)
            properties = lts.suspension.rollProperties(FAUmaxTauV2());
            testCase.verifyGreaterThan(properties.frontRollMomentFraction, 0.63);
            testCase.verifyLessThan(properties.frontRollMomentFraction, 0.66);
        end

        function frontAntiRollBarRaisesFrontFraction(testCase)
            vehicle = FAUmaxTauV2();
            baseline = lts.suspension.rollProperties(vehicle);
            vehicle.suspension.front.antiRollStiffness_NmPerRad = 20e3;
            modified = lts.suspension.rollProperties(vehicle);
            testCase.verifyGreaterThan(modified.frontRollMomentFraction, ...
                baseline.frontRollMomentFraction);
        end

        function applicationLeavesInputAndSetsCopy(testCase)
            vehicle = FAUmaxTauV2();
            [configuredVehicle, properties] = ...
                lts.suspension.applyRollStiffness(vehicle);
            testCase.verifyEqual(vehicle.suspension.frontRollMomentFraction, 0.50);
            testCase.verifyEqual(...
                configuredVehicle.suspension.frontRollMomentFraction, ...
                properties.frontRollMomentFraction);
            testCase.verifyEqual(...
                configuredVehicle.suspension.frontRollStiffness_NmPerRad, ...
                properties.frontAxleRollStiffness_NmPerRad);
        end
    end
end
