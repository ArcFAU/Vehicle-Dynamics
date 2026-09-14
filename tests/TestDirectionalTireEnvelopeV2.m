classdef TestDirectionalTireEnvelopeV2 < matlab.unittest.TestCase
    methods (Test)
        function fittedEnvelopePreservesDirectionalPeaks(testCase)
            vehicle = FAUmaxTauV2();
            envelope = lts.tire.createPeakEnvelope(vehicle);
            peaks = lts.tire.peakForces(envelope, [400, 800], [0, 0]);
            testCase.verifyNotEqual(peaks.longitudinal_N, peaks.lateral_N);
            testCase.verifyGreaterThan(peaks.longitudinal_N, 0);
            testCase.verifyGreaterThan(peaks.lateral_N, 0);
            testCase.verifyNotEqual(peaks.lateral_N(1) / 400, ...
                peaks.lateral_N(2) / 800);
        end

        function combinedUseConsumesRemainingLongitudinalForce(testCase)
            vehicle = FAUmaxTauV2();
            envelope = lts.tire.createPeakEnvelope(vehicle);
            peaks = lts.tire.peakForces(envelope, 600, 0);
            pure = lts.tire.directionalLongitudinalCapacity(...
                envelope, 600, 0, 0);
            combined = lts.tire.directionalLongitudinalCapacity(...
                envelope, 600, 0.8 * peaks.lateral_N, 0);
            testCase.verifyEqual(pure, peaks.longitudinal_N, RelTol=1e-12);
            testCase.verifyLessThan(combined, pure);
        end

        function drivenWheelCountLimitsPointMassTraction(testCase)
            rear = FAUmaxTauV2("singleRearMotor");
            allWheel = FAUmaxTauV2("fourMotorAWD");
            rear = vehicleValue(rear, 1e6);
            allWheel = vehicleValue(allWheel, 1e6);
            rearModel = lts.model.pointMassAero.create(rear);
            awdModel = lts.model.pointMassAero.create(allWheel);
            rearLimit = rearModel.accelerationLimits(1, 0);
            awdLimit = awdModel.accelerationLimits(1, 0);
            testCase.verifyGreaterThan(awdLimit.maximumAcceleration_mps2, ...
                rearLimit.maximumAcceleration_mps2);
        end
    end
end

function vehicle = vehicleValue(vehicle, power_W)
vehicle.powertrain.maximumMotorTorque_Nm = 1e6;
vehicle.powertrain.maximumPower_W = power_W;
vehicle.powertrain.maximumMotorPower_W = power_W ...
    / vehicle.powertrain.motorCount;
vehicle.powertrain.driveLimitModel.mode = 'constant';
end
