classdef TestEventsScoringAndSensitivity < matlab.unittest.TestCase
    methods (Test)
        function timeCurveMatchesFsgEndpoints(testCase)
            references = lts.config.scoring.referenceTemplate();
            references.acceleration.bestTime_s = 4;
            rules = lts.config.scoring.fsg2026(references);
            result = TestEventsScoringAndSensitivity.eventResult( ...
                'acceleration', 'manual', 4);
            score = lts.scoring.scoreEvents(result, rules, ...
                IncludeDriverless=false);
            testCase.verifyEqual(score.totalPoints, 50, 'AbsTol', 1e-12);

            result.time_s = 1.7 * 4;
            score = lts.scoring.scoreEvents(result, rules, ...
                IncludeDriverless=false);
            testCase.verifyEqual(score.totalPoints, 2.5, 'AbsTol', 1e-12);
        end

        function driverlessRankingCanUseAnnualResults(testCase)
            references = lts.config.scoring.referenceTemplate();
            references.dvAcceleration.competitorTimes_s = [3.8, 4.1, 5.0];
            rules = lts.config.scoring.fsg2026(references);
            result = TestEventsScoringAndSensitivity.eventResult( ...
                'dvAcceleration', 'driverless', 4.5);
            score = lts.scoring.scoreEvents(result, rules, ...
                IncludeDriverless=true);
            testCase.verifyEqual(score.totalPoints, 37.5, 'AbsTol', 1e-12);
            testCase.verifyTrue(score.isComplete);
        end

        function missingAnnualDataIsExplicit(testCase)
            rules = lts.config.scoring.fsg2026();
            result = TestEventsScoringAndSensitivity.eventResult( ...
                'acceleration', 'manual', 4.5);
            score = lts.scoring.scoreEvents(result, rules, ...
                IncludeDriverless=false);
            testCase.verifyTrue(isnan(score.totalPoints));
            testCase.verifyFalse(score.isComplete);
            testCase.verifyEqual(score.scoredPointsSubtotal, 0);
        end

        function missingScoringRuleIsRejected(testCase)
            rules.meta.organizer = 'Test';
            rules.meta.year = 1;
            rules.events = TestEventsScoringAndSensitivity.timeRule();
            result = TestEventsScoringAndSensitivity.eventResult( ...
                'unconfiguredEvent', 'manual', 4.5);
            testCase.verifyError(@() lts.scoring.scoreEvents( ...
                result, rules, IncludeDriverless=false), ...
                'lts:scoring:MissingRule');
        end

        function requestedEnergyCouplingCannotFallBack(testCase)
            temporaryFile = [tempname, '.mat'];
            cleanup = onCleanup(@() delete(temporaryFile)); %#ok<NASGU>
            A = [(0:10:50).', zeros(6, 1)];
            save(temporaryFile, 'A');
            event = lts.events.createDefinition(Id="test", Name="Test", ...
                Discipline="acceleration", TrackFile=temporaryFile, ...
                IsClosed=false, FinalSpeed_mps=100);
            testCase.verifyError(@() lts.events.runSet(event, ...
                TestEventsScoringAndSensitivity.model(), ...
                IncludeDriverless=false, CoupleEnergy=true), ...
                'lts:events:EnergyCouplingUnavailable');
        end

        function componentMassUpdatesTotalMass(testCase)
            vehicle = FAUmaxTauV2();
            definition = lts.study.parameter(Name="Car mass", ...
                Path="mass.carWithoutDriver_kg", Values=170);
            updated = lts.study.applyParameter(vehicle, definition, 170);
            testCase.verifyEqual(updated.mass.total_kg, 250);
            testCase.verifyEqual(vehicle.mass.total_kg, 250);
        end

        function eventTimingAndSweepRunEndToEnd(testCase)
            temporaryFile = [tempname, '.mat'];
            cleanup = onCleanup(@() delete(temporaryFile));
            A = [(0:10:50).', zeros(6, 1)];
            save(temporaryFile, 'A');
            event = lts.events.createDefinition(Id="test", Name="Test", ...
                Discipline="acceleration", TrackFile=temporaryFile, ...
                IsClosed=false, FinalSpeed_mps=100);
            rule.meta.organizer = 'Test';
            rule.meta.year = 1;
            rule.events = TestEventsScoringAndSensitivity.timeRule();
            parameter = lts.study.parameter(Name="Mass", ...
                Path="mass.total_kg", Values=[230, 240], Baseline=240);
            study = lts.study.runParameterSweep(FAUmaxTauV2(), ...
                parameter, event, @(~) TestEventsScoringAndSensitivity.model(), ...
                rule, IncludeDriverless=false, CoupleEnergy=false);
            testCase.verifySize(study.parameterValues, [2, 1]);
            testCase.verifyEqual(numel(study.cases), 2);
            testCase.verifyTrue(all(isfinite(study.totalPoints)));
            testCase.verifySize(study.penalty.eventTime_s, [2, 1]);
            testCase.verifyEqual(min(study.penalty.eventTime_s), 0, ...
                'AbsTol', 1e-12);
            testCase.verifyGreaterThanOrEqual( ...
                min(study.penalty.totalPointsLoss), 0);
            testCase.verifySize(study.relative.eventTime_percent, [2, 1]);
            testCase.verifyEqual(study.relative.eventTime_percent(2), 0, ...
                'AbsTol', 1e-12);
        end

        function configuredAeroMapKeepsAxleCoefficientsCoupled(testCase)
            projectRoot = fileparts(fileparts(mfilename('fullpath')));
            configPath = fullfile(projectRoot, 'config');
            addpath(configPath);
            cleanup = onCleanup(@() rmpath(configPath)); %#ok<NASGU>
            config = sensitivityStudyCase(Study="aeroMap", ...
                Model="dualTrackEnvelope", PowertrainLayout="fourMotorAWD", ...
                TireModel="fittedCombinedSlip", IncludeDriverless=false, ...
                CoupleEnergy=false, CreatePlots=false);
            vehicle = config.baseVehicle;
            vehicle = lts.study.applyParameter(vehicle, ...
                config.parameters(1), -4.8);
            vehicle = lts.study.applyParameter(vehicle, ...
                config.parameters(2), 0.60);

            testCase.verifyEqual(vehicle.aerodynamics.liftCoefficientFront, ...
                -4.8 * 0.60, 'AbsTol', 1e-12);
            testCase.verifyEqual(vehicle.aerodynamics.liftCoefficientRear, ...
                -4.8 * 0.40, 'AbsTol', 1e-12);
        end
    end

    methods (Static, Access = private)
        function result = eventResult(id, mode, time_s)
            result.id = id;
            result.name = id;
            result.discipline = id;
            result.mode = mode;
            result.time_s = time_s;
            result.lapResult = struct();
            result.track = struct();
        end

        function rule = timeRule()
            rule.eventId = 'test';
            rule.name = 'Test';
            rule.mode = 'manual';
            rule.method = 'timeCurve';
            rule.maximumPoints = 100;
            rule.cutoffFactor = 2;
            rule.minimumPointsFraction = 0;
            rule.exponent = 2;
            rule.reference = struct('bestTime_s', 10);
        end

        function model = model()
            model.name = 'Constant test model';
            model.level = 'test';
            model.maximumSpeedForCurvature = @(curvature) ...
                20 * ones(size(curvature));
            model.accelerationLimits = @(speed, lateral) struct( ...
                'maximumAcceleration_mps2', 5 * ones(size(speed)), ...
                'maximumDeceleration_mps2', 5 * ones(size(speed)), ...
                'feasible', true(size(lateral)));
        end
    end
end
