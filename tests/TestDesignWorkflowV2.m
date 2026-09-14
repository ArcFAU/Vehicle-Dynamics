classdef TestDesignWorkflowV2 < matlab.unittest.TestCase
    methods (Test)
        function registryHasUniqueExplicitModelIds(testCase)
            definitions = lts.model.registry();
            ids = [definitions.id];
            testCase.verifyEqual(numel(unique(ids)), numel(ids));
            testCase.verifyEqual(numel(ids), 9);
            testCase.verifyTrue(all(strlength(ids) > 0));
            testCase.verifyTrue(all(isfield(definitions, ...
                {'mapType', 'tireRequirement', 'supportsYawAudit', ...
                'operatingPointType'})));
        end

        function availableConfigurationsComesFromRegistry(testCase)
            configurations = lts.model.availableConfigurations(...
                Strategy="trackLocal");
            testCase.verifyEqual(height(configurations), 3);
            testCase.verifyTrue(all(configurations.strategy == "trackLocal"));
            testCase.verifyTrue(all(configurations.supportsYawAudit));
            fitted = lts.model.availableConfigurations(...
                Model="dualTrackSlipFitted", Strategy="mapped");
            testCase.verifyEqual(fitted.tireRequirement, ...
                "fittedCombinedSlip");
            testCase.verifyTrue(fitted.mapRequired);
        end

        function validateCaseReturnsDryRunSummary(testCase)
            vehicle = FAUmaxTauV2();
            track.meta.name = 'Validation track';
            track.distance_m = [0; 10; 20];
            track.curvature_1pm = [0; 0.01; 0];
            track.isClosed = false;
            config = lts.config.createCase(Vehicle=vehicle, ...
                Model="dualTrackEnvelope", Strategy="direct", ...
                IncludeEnergy=false, IncludeYawInertia=false, ...
                MaximumSpeed_mps=100, InitialSpeed_mps=0, ...
                FinalSpeed_mps=Inf, ShowSummary=false);
            summary = lts.config.validateCase(config, track, Display=false);
            testCase.verifyEqual(summary.modelId, "dualTrackEnvelope");
            testCase.verifyEqual(summary.workload, "direct evaluation");
            testCase.verifyEqual(summary.track, "Validation track");
        end

        function createCaseDerivesLevelFromModelId(testCase)
            vehicle = FAUmaxTauV2();
            config = lts.config.createCase(Vehicle=vehicle, ...
                Model="dualTrackEnvelope", Strategy="direct", ...
                IncludeEnergy=false, IncludeYawInertia=false, ...
                MaximumSpeed_mps=100, InitialSpeed_mps=0, ...
                FinalSpeed_mps=Inf);
            testCase.verifyEqual(config.modelId, 'dualTrackEnvelope');
            testCase.verifyEqual(config.level, '2.0');
        end

        function createCaseRejectsImplicitOrInvalidSelection(testCase)
            vehicle = FAUmaxTauV2();
            incomplete = @() lts.config.createCase(Vehicle=vehicle, ...
                Strategy="direct", IncludeEnergy=false, ...
                IncludeYawInertia=false, MaximumSpeed_mps=100, ...
                InitialSpeed_mps=0, FinalSpeed_mps=Inf);
            invalidCombination = @() lts.config.createCase(...
                Vehicle=vehicle, Model="dualTrackEnvelope", ...
                Strategy="trackLocal", IncludeEnergy=false, ...
                IncludeYawInertia=false, MaximumSpeed_mps=100, ...
                InitialSpeed_mps=0, FinalSpeed_mps=Inf);
            testCase.verifyError(incomplete, ...
                'lts:config:MissingSelection');
            testCase.verifyError(invalidCombination, ...
                'lts:config:ModelStrategy');
        end

        function capabilityMatrixKeepsPointMassLevelZero(testCase)
            capabilities = lts.design.capabilityMatrix();
            testCase.verifyTrue(any(capabilities.topic == "mass"));
            testCase.verifyTrue(any(startsWith(capabilities.minimumLevel, "0")));
            torqueRow = capabilities.topic == "torqueVectoring";
            testCase.verifyEqual(capabilities.minimumLevel(torqueRow), "2.0");
        end

        function levelZeroCaseProducesDecisionOutputs(testCase)
            vehicle = FAUmaxTauV2();
            track.meta.name = 'V2 circle';
            track.distance_m = linspace(0, 2*pi*20, 101)';
            track.curvature_1pm = ones(101, 1) / 20;
            track.isClosed = true;
            config = lts.config.createCase(Vehicle=vehicle, ...
                Model="pointMassAero", ...
                Strategy="direct", IncludeEnergy=true, ...
                IncludeYawInertia=false, MaximumSpeed_mps=100, ...
                InitialSpeed_mps=0, FinalSpeed_mps=Inf);
            result = lts.design.runCase(config, track);
            testCase.verifyGreaterThan(result.lap.lapTime_s, 0);
            testCase.verifyGreaterThanOrEqual(...
                result.brakeDemand.maximumTotalForce_N, 0);
            testCase.verifyTrue(isfield(result, 'energy'));
            testCase.verifyTrue(isfield(result, 'limitClassification'));
            testCase.verifyEqual(result.meta.configuration.schema, ...
                'lts.case.v1');
            testCase.verifyEqual(result.meta.modelId, 'pointMassAero');
            testCase.verifyEqual(result.meta.physics.energy, ...
                'postProcessed');
            testCase.verifyEqual(result.meta.physics.yawInertia, ...
                'notIncluded');
            testCase.verifyEqual(numel(result.limitClassification.primary), ...
                numel(result.lap.segmentAcceleration_mps2));
        end

        function coverageCollectsMultipleCases(testCase)
            lap.distance_m = [0; 1; 2];
            lap.speed_mps = [2; 3; 2];
            lap.segmentAcceleration_mps2 = [1; -1];
            lap.lateralAcceleration_mps2 = [0; 2; 0];
            caseResult.lap = lap;
            coverage = lts.analysis.operatingPointCoverage(...
                {caseResult, caseResult});
            testCase.verifyEqual(coverage.meta.caseCount, 2);
            testCase.verifyEqual(coverage.meta.sampleCount, 4);
        end
    end
end
