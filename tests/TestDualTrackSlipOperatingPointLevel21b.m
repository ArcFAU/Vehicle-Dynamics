classdef TestDualTrackSlipOperatingPointLevel21b < matlab.unittest.TestCase
    methods (Test)
        function straightOperatingPointClosesEquilibrium(testCase)
            vehicle = FAUmaxTauV2();
            point = lts.model.dualTrackSlip.operatingPoint(vehicle, 20, 0, 0);
            testCase.verifyTrue(point.feasible);
            testCase.verifyLessThan(abs(point.residuals.longitudinalForce_N), 1e-3);
            testCase.verifyLessThan(abs(point.residuals.lateralForce_N), 1e-3);
            testCase.verifyLessThan(abs(point.residuals.yawMoment_Nm), 1e-3);
        end

        function leftAndRightTurnsMirror(testCase)
            vehicle = FAUmaxTauV2();
            left = lts.model.dualTrackSlip.operatingPoint(vehicle, 15, 0, 4);
            right = lts.model.dualTrackSlip.operatingPoint(vehicle, 15, 0, -4);
            testCase.verifyTrue(left.feasible);
            testCase.verifyTrue(right.feasible);
            testCase.verifyEqual(left.steeringAngle_rad, ...
                -right.steeringAngle_rad, 'AbsTol', 5e-4);
            testCase.verifyEqual(left.bodySlipAngle_rad, ...
                -right.bodySlipAngle_rad, 'AbsTol', 5e-4);
            testCase.verifyEqual(left.normalLoads_N, ...
                right.normalLoads_N([2, 1, 4, 3]), 'AbsTol', 1e-10);
        end

        function rearDriveLeavesFrontLongitudinalForcesAtZero(testCase)
            vehicle = FAUmaxTauV2("dualRearMotors");
            point = lts.model.dualTrackSlip.operatingPoint(vehicle, 15, 1, 2);
            testCase.verifyTrue(point.feasible);
            testCase.verifyEqual(point.wheelLongitudinalForces_N(1:2), ...
                [0, 0], 'AbsTol', 1e-3);
            testCase.verifyGreaterThan(min(point.wheelLongitudinalForces_N(3:4)), 0);
        end

        function conditionalRestartEscapesLocalMinimum(testCase)
            vehicle = lts.chassis.applyTorsionalStiffness(...
                FAUmaxTauV2(), 2000 / deg2rad(1));
            point = lts.model.dualTrackSlip.operatingPoint(vehicle, ...
                15.4935, -7.9872, 7.6593);
            testCase.verifyTrue(point.feasible);
            testCase.verifyGreaterThan(point.solver.attemptCount, 1);
            testCase.verifyLessThan(norm(...
                point.solver.normalizedResidual, Inf), 1e-6);
        end
    end
end
