classdef TestEnergyLevel24 < matlab.unittest.TestCase
    methods (Test)
        function constantSpeedNeedsOnlyRoadLoadsAndAuxiliaries(testCase)
            vehicle = TestEnergyLevel24.losslessVehicle();
            vehicle.energy.rollingResistanceCoefficient = 0;
            vehicle.aerodynamics.dragCoefficient = 0;
            vehicle.energy.auxiliaryPower_W = 1000;
            lap = TestEnergyLevel24.lap([10, 10], [0, 10], [0, 1]);
            energy = lts.energy.analyzeLap(lap, vehicle);
            testCase.verifyEqual(energy.netBatteryEnergy_kWh, ...
                1000 / 3.6e6, 'AbsTol', 1e-12);
        end

        function losslessAccelerationAndBrakingRecoverKineticEnergy(testCase)
            vehicle = TestEnergyLevel24.losslessVehicle();
            lap = TestEnergyLevel24.lap([0, 10, 0], [0, 10, 20], [0, 2, 4]);
            energy = lts.energy.analyzeLap(lap, vehicle);
            expected_kWh = 0.5 * vehicle.mass.total_kg * 10 ^ 2 / 3.6e6;
            testCase.verifyEqual(energy.tractionBatteryEnergy_kWh, ...
                expected_kWh, 'RelTol', 1e-12);
            testCase.verifyEqual(energy.recoveredEnergy_kWh, ...
                expected_kWh, 'RelTol', 1e-12);
            testCase.verifyEqual(energy.netBatteryEnergy_kWh, 0, ...
                'AbsTol', 1e-12);
        end

        function regenLimitCreatesFrictionBrakeEnergy(testCase)
            vehicle = TestEnergyLevel24.losslessVehicle();
            vehicle.energy.maximumMechanicalRegenPower_W = 100;
            lap = TestEnergyLevel24.lap([10, 0], [0, 10], [0, 1]);
            energy = lts.energy.analyzeLap(lap, vehicle);
            testCase.verifyGreaterThan(energy.frictionBrakeEnergy_kWh, 0);
            testCase.verifyLessThan(energy.regenPotentialUtilization, 1);
        end

        function repetitionsAndCapacityReserveAreApplied(testCase)
            vehicle = TestEnergyLevel24.losslessVehicle();
            vehicle.energy.auxiliaryPower_W = 3600;
            vehicle.energy.usableSocWindow = 0.8;
            vehicle.energy.finishEnergyReserveFraction = 0.1;
            lap = TestEnergyLevel24.lap([10, 10], [0, 10], [0, 1]);
            energy = lts.energy.analyzeLap(lap, vehicle, Repetitions=10);
            testCase.verifyEqual(energy.netBatteryEnergy_kWh, 0.01, ...
                'AbsTol', 1e-12);
            testCase.verifyEqual(energy.requiredInstalledCapacity_kWh, ...
                0.01 / (0.72 * 0.9), 'AbsTol', 1e-12);
        end

        function fsgEfficiencyScoreUsesTimeSquaredEnergyFactor(testCase)
            references = lts.config.scoring.referenceTemplate();
            references.endurance.bestTime_s = 1000;
            references.efficiency.minimumFactor_s2kWh = 1e6;
            rules = lts.config.scoring.fsg2026(references);
            result = TestEnergyLevel24.enduranceResult(1000, 1);
            score = lts.scoring.scoreEvents(result, rules, ...
                IncludeDriverless=false);
            efficiency = score.events(strcmp({score.events.eventId}, ...
                'efficiency'));
            testCase.verifyEqual(efficiency.points, 75, 'AbsTol', 1e-12);
            testCase.verifyEqual(efficiency.efficiencyFactor_s2kWh, 1e6);
        end


        function driveLimitCanDerateAtSelectedSoc(testCase)
            vehicle = FAUmaxTauV2();
            vehicle.powertrain.driveLimitModel.evaluationSoc = 0.5;
            nominal = lts.powertrain.maximumDriveForce( ...
                vehicle.powertrain, 20);
            vehicle.powertrain.driveLimitModel.evaluationSoc = 0.05;
            lowSoc = lts.powertrain.maximumDriveForce( ...
                vehicle.powertrain, 20);
            testCase.verifyLessThan(lowSoc.maximumForce_N, ...
                nominal.maximumForce_N);
        end

        function highSocCanDisableRegenIndependently(testCase)
            vehicle = TestEnergyLevel24.losslessVehicle();
            lap = TestEnergyLevel24.lap([10, 0], [0, 10], [0, 2]);
            vehicle.energy.initialSoc = 0.5;
            available = lts.energy.analyzeLap(lap, vehicle);
            vehicle.energy.initialSoc = 1.0;
            vehicle.energy.socLimitModel.regenPowerScale(end) = 0;
            vehicle.energy.socLimitModel.regenTorqueScale(end) = 0;
            blocked = lts.energy.analyzeLap(lap, vehicle);
            testCase.verifyGreaterThan(available.recoveredEnergy_kWh, 0);
            testCase.verifyEqual(blocked.recoveredEnergy_kWh, 0, ...
                'AbsTol', 1e-12);
        end

        function componentMapInterfaceInterpolatesEfficiencies(testCase)
            vehicle = FAUmaxTauV2();
            model = vehicle.energy.efficiencyModel;
            model.mode = 'map';
            model.motor.speed_rpm = [0, 20000];
            model.motor.torque_Nm = [0, 100];
            model.motor.driveValues = 0.90 * ones(2);
            model.motor.regenValues = 0.80 * ones(2);
            model.inverter.power_W = [0, 100e3];
            model.inverter.soc = [0, 1];
            model.inverter.driveValues = 0.95 * ones(2);
            model.inverter.regenValues = 0.94 * ones(2);
            model.battery.power_W = [0, 100e3];
            model.battery.soc = [0, 1];
            model.battery.dischargeValues = 0.98 * ones(2);
            model.battery.chargeValues = 0.97 * ones(2);
            vehicle.energy.efficiencyModel = model;
            efficiency = lts.energy.conversionEfficiencies(vehicle, ...
                10, 1000, 0.5, "drive");
            expected = vehicle.powertrain.gearboxEfficiency ...
                * 0.90 * 0.95 * 0.98;
            testCase.verifyEqual(efficiency.total, expected, ...
                'AbsTol', 1e-12);
        end


        function coupledEnergySolverWorksWithSingleTrack(testCase)
            vehicle = FAUmaxTauV2();
            vehicle.energy.maximumBatteryDischargePower_W = 8e3;
            vehicle.energy.auxiliaryPower_W = 0;
            vehicle.energy.initialSoc = 0.8;
            vehicle.energy.installedCapacity_kWh = 20;
            vehicle.powertrain.driveLimitModel.powerScale(:) = 1;
            model = lts.model.singleTrackAero.create(vehicle);
            track = TestEnergyLevel24.straightTrack();
            uncoupled = lts.solver.solveLap(track, model, ...
                initialSpeed_mps=0, finalSpeed_mps=100);
            coupled = lts.solver.solveLapWithEnergy(track, model, ...
                InitialSoc=0.8, InitialSpeed_mps=0, FinalSpeed_mps=100);
            testCase.verifyGreaterThan(coupled.lapTime_s, ...
                uncoupled.lapTime_s);
            testCase.verifyEqual(coupled.meta.baseModelLevel, '1.2');
            testCase.verifyLessThan(coupled.energy.finalSoc, 0.8);
        end
    end

    methods (Static, Access = private)
        function vehicle = losslessVehicle()
            vehicle = FAUmaxTauV2();
            vehicle.aerodynamics.dragCoefficient = 0;
            vehicle.aerodynamics.liftCoefficientFront = 0;
            vehicle.aerodynamics.liftCoefficientRear = 0;
            vehicle.energy.rollingResistanceCoefficient = 0;
            vehicle.energy.motorEfficiency = 1;
            vehicle.energy.regenMotorEfficiency = 1;
            vehicle.energy.inverterEfficiency = 1;
            vehicle.energy.batteryDischargeEfficiency = 1;
            vehicle.energy.batteryChargeEfficiency = 1;
            vehicle.powertrain.gearboxEfficiency = 1;
            vehicle.energy.auxiliaryPower_W = 0;
            vehicle.energy.maximumMechanicalRegenPower_W = 1e9;
            vehicle.energy.maximumRegenMotorTorque_Nm = 1e9;
            vehicle.energy.maximumRegenMotorPower_W = 1e9;
            vehicle.energy.maximumBatteryChargePower_W = 1e9;
            vehicle.energy.maximumRegenDeceleration_mps2 = 100;
            vehicle.energy.minimumRegenSpeed_mps = 0;
            vehicle.energy.initialSoc = 0.5;
            vehicle.energy.socLimitModel.chargePowerScale(:) = 1;
            vehicle.energy.socLimitModel.regenTorqueScale(:) = 1;
            vehicle.energy.socLimitModel.regenPowerScale(:) = 1;
            vehicle.powertrain.driveLimitModel.powerScale(:) = 1;
        end

        function lap = lap(speed_mps, distance_m, time_s)
            lap.distance_m = distance_m(:);
            lap.time_s = time_s(:);
            lap.speed_mps = speed_mps(:);
            lap.segmentAcceleration_mps2 = ...
                diff(lap.speed_mps .^ 2) ./ (2 * diff(lap.distance_m));
        end

        function result = enduranceResult(time_s, energy_kWh)
            result.id = 'endurance';
            result.name = 'Endurance';
            result.discipline = 'endurance';
            result.mode = 'manual';
            result.time_s = time_s;
            result.energy.status = 'calculated';
            result.energy.netBatteryEnergy_kWh = energy_kWh;
            result.lapResult = struct();
            result.track = struct();
        end


        function track = straightTrack()
            track.meta.name = 'Synthetic straight';
            track.meta.source = 'unit test';
            track.distance_m = (0:5:75).';
            track.curvature_1pm = zeros(size(track.distance_m));
            track.isClosed = false;
        end
    end
end
