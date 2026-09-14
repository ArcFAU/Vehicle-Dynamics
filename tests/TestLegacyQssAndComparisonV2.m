classdef TestLegacyQssAndComparisonV2 < matlab.unittest.TestCase
    methods (Test)
        function legacyQssRunsAtLevelsZeroAndOne(testCase)
            vehicle = FAUmaxTauV2();
            track.distance_m = linspace(0, 2*pi*20, 61)';
            track.curvature_1pm = ones(61, 1) / 20;
            track.isClosed = true;
            pointConfig = lts.config.createCase(Vehicle=vehicle, ...
                Model="pointMassAero", Strategy="legacyQss", IncludeEnergy=false, ...
                IncludeYawInertia=false, MaximumSpeed_mps=100, ...
                InitialSpeed_mps=0, FinalSpeed_mps=Inf);
            singleConfig = lts.config.createCase(Vehicle=vehicle, ...
                Model="singleTrackAero", Strategy="legacyQss", IncludeEnergy=false, ...
                IncludeYawInertia=false, MaximumSpeed_mps=100, ...
                InitialSpeed_mps=0, FinalSpeed_mps=Inf);
            pointMass = lts.design.runCase(pointConfig, track);
            singleTrack = lts.design.runCase(singleConfig, track);
            testCase.verifyEqual(pointMass.meta.strategy, 'legacyQss');
            testCase.verifyEqual(singleTrack.meta.strategy, 'legacyQss');
            testCase.verifyGreaterThan(pointMass.lap.lapTime_s, 0);
            testCase.verifyGreaterThan(singleTrack.lap.lapTime_s, 0);
        end

        function legacyAdapterAndComparatorReportKnownOffset(testCase)
            distance = (0:10)';
            speed = 10 + zeros(size(distance));
            curvature = zeros(size(distance));
            time = distance / 10;
            reference = lts.analysis.adaptLegacyResult(...
                distance, speed, curvature, time);
            candidate = reference;
            candidate.lap.time_s = candidate.lap.time_s + ...
                0.1 * candidate.lap.time_s / candidate.lap.time_s(end);
            candidate.lap.lapTime_s = candidate.lap.time_s(end);
            comparison = lts.analysis.compareResults(reference, candidate);
            testCase.verifyEqual(comparison.lapTimeDifference_s, 0.1, ...
                AbsTol=1e-12);
            testCase.verifyEqual(comparison.speedRmsDifference_mps, 0, ...
                AbsTol=1e-12);
        end
    end
end
