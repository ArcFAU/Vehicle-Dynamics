classdef TestTransparentV2 < matlab.unittest.TestCase
    methods (Test)
        function registryContainsExactlyThreeFamilies(testCase)
            models = lts.v2.modelRegistry();
            testCase.verifyEqual(string({models.id}), ...
                ["pointMass", "singleTrack", "dualTrack"]);
        end

        function missingFeatureIsRejected(testCase)
            input = testInput();
            input.features = rmfield(input.features, 'aero');
            testCase.verifyError(@() lts.v2.createCase(input), ...
                'lts:v2:MissingInput');
        end

        function aeroOffIsEffectiveAndVisible(testCase)
            input = testInput();
            input.features.aero = false;
            config = lts.v2.createCase(input);
            [config, report] = lts.v2.validateCase(config, Display=false);
            testCase.verifyEqual(config.effectiveVehicle.aerodynamics.dragCoefficient, 0);
            testCase.verifyEqual(config.effectiveVehicle.aerodynamics.totalLiftCoefficient, 0);
            aeroRows = startsWith(report.inputs.path, "vehicle.aerodynamics.");
            testCase.verifyTrue(all(report.inputs.status(aeroRows) == "ignored"));
            chassisRows = startsWith(report.inputs.path, "vehicle.chassis.");
            testCase.verifyTrue(all(report.inputs.status(chassisRows) ...
                == "notSupported"));
        end

        function unsupportedCombinationIsRejected(testCase)
            input = testInput();
            input.model = "singleTrack";
            input.features.loadTransfer = false;
            testCase.verifyError(@() lts.v2.createCase(input), ...
                'lts:v2:UnsupportedFeature');
        end

        function pointMassLoadTransferMappingIsVisible(testCase)
            input = testInput();
            input.features.loadTransfer = true;
            config = lts.v2.createCase(input);
            testCase.verifyEqual(config.legacyModelId, 'pointMassAero');
            testCase.verifyEqual(config.legacyStrategy, 'legacyQss');
        end

        function selectedDualTrackTireModelChoosesKernel(testCase)
            input = testInput();
            input.model = "dualTrack";
            input.features.loadTransfer = true;
            input.features.tireSlip = true;
            input.solver.strategy = "trackLocal";
            input.vehicle = lts.tire.selectModel(input.vehicle, "tmeasyAdtm");
            config = lts.v2.createCase(input);
            testCase.verifyEqual(config.legacyModelId, 'dualTrackSlipTMeasy');
        end

        function resultStoresExecutedConfiguration(testCase)
            input = testInput();
            config = lts.v2.createCase(input);
            result = evalcResult(@() lts.v2.runCase(config));
            testCase.verifyEqual(result.meta.interface, 'transparent V2');
            testCase.verifyEqual(result.meta.modelFamily, 'pointMass');
            testCase.verifyTrue(isfield(result.meta, 'inputReport'));
            testCase.verifyEqual(result.meta.effectiveConfiguration.schema, ...
                'lts.transparent.v2');
        end
    end
end

function input = testInput()
input.vehicle = FAUmaxTauV2();
input.track.meta.name = 'V2 test track';
input.track.distance_m = [0; 10; 20];
input.track.curvature_1pm = [0; 0; 0];
input.track.isClosed = false;
input.model = "pointMass";
input.features = struct('aero', true, 'loadTransfer', false, ...
    'tireSlip', false, 'energy', false, 'yawInertia', false, ...
    'chassisTorsion', false);
input.solver.strategy = "direct";
input.solver.maximumSpeed_mps = 50;
input.solver.initialSpeed_mps = 0;
input.solver.finalSpeed_mps = 0;
input.solver.performanceMap = [];
input.solver.trackLocal = struct('speedRows', 3, ...
    'lateralFractions', [], 'boundaryIterations', 3, ...
    'interpolationSafetyFactor', NaN, 'showProgress', false, ...
    'auditPointCount', 2);
end

function result = evalcResult(functionHandle)
evalc('result = functionHandle();');
end
