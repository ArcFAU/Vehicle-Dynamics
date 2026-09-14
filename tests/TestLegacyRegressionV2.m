classdef TestLegacyRegressionV2 < matlab.unittest.TestCase
    properties (Constant)
        LegacyVehicleFile = ...
            'C:\Octanes\SVN\LaptimeSim\RundenzeitSimulation\Fahrzeuge\Tau.mat'
    end

    methods (Test)
        function adapterCreatesVerifiedRearDriveVehicle(testCase)
            testCase.assumeTrue(isfile(testCase.LegacyVehicleFile));
            data = load(testCase.LegacyVehicleFile, 'C');
            legacy = applyReferenceOverrides(data.C);
            [vehicle, mapping] = lts.compatibility.mapLegacyVehicle(legacy);
            testCase.verifyEqual(vehicle.powertrain.layout, 'singleRearMotor');
            testCase.verifyEqual(vehicle.suspension.frontCamber_rad, 0);
            testCase.verifyTrue(mapping.tireDatasetVerified);
            testCase.verifyTrue(mapping.directionalPeakEnvelope);
        end

        function tauAccelerationRemainsCloseToLegacy(testCase)
            testCase.assumeTrue(isfile(testCase.LegacyVehicleFile));
            data = load(testCase.LegacyVehicleFile, 'C');
            vehicle = lts.compatibility.mapLegacyVehicle( ...
                applyReferenceOverrides(data.C));
            tracks = lts.io.trackCatalog();
            track = lts.io.loadTrackMatrix(string(tracks.acceleration), false);
            pointMass = lts.design.runCase(caseFor(vehicle, "0.3"), track);
            singleTrack = lts.design.runCase(caseFor(vehicle, "1.2"), track);
            testCase.verifyLessThan(relativeDifference( ...
                pointMass.lap.lapTime_s, 4.120677031888), 0.01);
            testCase.verifyLessThan(relativeDifference( ...
                singleTrack.lap.lapTime_s, 4.132691503121), 0.01);
        end

        function tauEnduranceStaysInsideDocumentedBands(testCase)
            testCase.assumeTrue(isfile(testCase.LegacyVehicleFile));
            data = load(testCase.LegacyVehicleFile, 'C');
            vehicle = lts.compatibility.mapLegacyVehicle( ...
                applyReferenceOverrides(data.C));
            tracks = lts.io.trackCatalog();
            track = lts.io.loadTrackMatrix(string(tracks.endurance), true);
            pointMass = lts.design.runCase(caseFor(vehicle, "0.3"), track);
            singleTrack = lts.design.runCase(caseFor(vehicle, "1.2"), track);
            testCase.verifyLessThan(relativeDifference( ...
                pointMass.lap.lapTime_s, 77.293274291252), 0.05);
            testCase.verifyLessThan(relativeDifference( ...
                singleTrack.lap.lapTime_s, 79.579323815202), 0.04);
        end
    end
end

function config = caseFor(vehicle, level)
modelId = "pointMassAero";
if level == "1.2"
    modelId = "singleTrackAero";
end
config = lts.config.createCase(Vehicle=vehicle, Model=modelId, ...
    Strategy="legacyQss", IncludeEnergy=false, IncludeYawInertia=false, ...
    MaximumSpeed_mps=100, InitialSpeed_mps=0, FinalSpeed_mps=Inf);
end

function legacy = applyReferenceOverrides(legacy)
legacy.aeroh = 0.4;
legacy.DRS = 0;
legacy.cr = 0;
legacy.Cw = 0;
legacy.quer = 0.48;
legacy.laengs = 0.50;
end

function difference = relativeDifference(actual, reference)
difference = abs(actual - reference) / reference;
end
