classdef TestPowertrainLayouts < matlab.unittest.TestCase
    methods (Test)
        function allLayoutsAreValid(testCase)
            names = ["singleFrontMotor", "singleRearMotor", ...
                "dualFrontMotors", "dualRearMotors", ...
                "dualMotorAWD", "fourMotorAWD"];
            for name = names
                vehicle = FAUmaxTauV2(name);
                testCase.verifyWarningFree(@() ...
                    lts.validation.validateVehicle(vehicle));
            end
        end

        function dualRearHasTwiceSingleRearTorqueForce(testCase)
            single = FAUmaxTauV2("singleRearMotor");
            dual = FAUmaxTauV2("dualRearMotors");
            singleDrive = lts.powertrain.maximumDriveForce(single.powertrain, 0);
            dualDrive = lts.powertrain.maximumDriveForce(dual.powertrain, 0);

            testCase.verifyEqual(dualDrive.maximumForce_N, ...
                2 * singleDrive.maximumForce_N, 'AbsTol', 1e-10);
        end

        function singleMotorPowerIsMotorLimited(testCase)
            vehicle = FAUmaxTauV2("singleRearMotor");
            drive = lts.powertrain.maximumDriveForce(vehicle.powertrain, 20);

            testCase.verifyEqual(drive.availableElectricalPower_W, ...
                vehicle.powertrain.maximumMotorPower_W);
        end

        function fourMotorAwdUsesSystemPowerLimit(testCase)
            vehicle = FAUmaxTauV2("fourMotorAWD");
            drive = lts.powertrain.maximumDriveForce(vehicle.powertrain, 20);

            testCase.verifyEqual(drive.availableElectricalPower_W, ...
                vehicle.powertrain.maximumPower_W);
        end

        function detailLimitsCanBeDisabled(testCase)
            vehicle = FAUmaxTauV2("singleRearMotor");
            vehicle.powertrain.maximumMotorPower_W = 1;
            vehicle.powertrain.maximumMotorTorque_Nm = 1;
            vehicle.powertrain.maximumMotorSpeed_rpm = 1;
            vehicle.powertrain.driveLimitModel.evaluationSoc = 0;
            options = vehicle.powertrain.limitOptions;
            options.motorPowerEnabled = false;
            options.motorTorqueEnabled = false;
            options.motorSpeedEnabled = false;
            options.socPowerScaleEnabled = false;
            vehicle.powertrain.limitOptions = options;

            drive = lts.powertrain.maximumDriveForce(vehicle.powertrain, 20);

            testCase.verifyEqual(drive.availableElectricalPower_W, ...
                vehicle.powertrain.maximumPower_W);
            testCase.verifyEqual(drive.socPowerScale, 1);
            testCase.verifyEqual(drive.maximumForce_N, ...
                drive.powerLimitedForce_N);
            testCase.verifyEqual(drive.maximumVehicleSpeed_mps, Inf);
        end

        function awdSplitsDriveForceBetweenAxles(testCase)
            vehicle = FAUmaxTauV2("fourMotorAWD");
            model = lts.model.dualTrackEnvelope.create(vehicle);
            point = model.operatingPoint(10, 1, 0);

            testCase.verifyEqual(sum(point.longitudinalForces_N(1:2)), ...
                sum(point.longitudinalForces_N(3:4)), 'AbsTol', 1e-10);
        end
    end
end
