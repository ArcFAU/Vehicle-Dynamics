classdef TestFsg2026SensitivityV9 < matlab.unittest.TestCase
    methods (Test)
        function eventSpecificDriverMassesAreRecorded(testCase)
            masses = struct('acceleration',70,'skidpad',80, ...
                'autocross',80,'endurance',90);
            report = runFsg2026SensitivityV9( ...
                ShowProgress=false,ValidationOnly=true, ...
                IncludeYawInertia=false,DriverMass_kg=masses);
            testCase.verifyEqual(report.driverMass_kg,masses);
            testCase.verifyEqual(report.yawTreatment,"postProcessingOnly");
            testCase.verifyTrue(isempty(report.yawGate));

            file = fullfile(report.cacheDirectory,'baseline.mat');
            stored = load(file,'simulation');
            sim = stored.simulation;
            testCase.verifyEqual(sim.driverMass_kg,masses);
            testCase.verifyEqual(sim.vehicle.mass.carWithoutDriver_kg,170);
            testCase.verifyEqual( ...
                sim.vehicle.energy.maximumRegenDeceleration_mps2,100);
            testCase.verifyEqual(numel(sim.eventResults),4);
            testCase.verifyTrue(sim.scoring.isComplete);
        end
    end
end
