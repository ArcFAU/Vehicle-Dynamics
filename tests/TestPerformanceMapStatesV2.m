classdef TestPerformanceMapStatesV2 < matlab.unittest.TestCase
    methods (Test)
        function mapStoresSolvedBoundaryStates(testCase)
            vehicle = FAUmaxTauV2();
            map = lts.model.dualTrackSlip.buildPerformanceMap(vehicle, ...
                speedGrid_mps=[5, 8], lateralFractions=[0, 1], ...
                boundaryIterations=2, interpolationSafetyFactor=0.9, ...
                showProgress=false, enableSolverRetries=true);
            testCase.verifyTrue(map.meta.boundaryStatesStored);
            testCase.verifySize(map.boundaryStates.lateral, [2, 2]);
            testCase.verifySize(map.boundaryStates.acceleration, [2, 2, 2]);
            states = map.boundaryStates.lateral;
            present = cellfun(@(item) ~isempty(item), states);
            testCase.verifyTrue(any(present, 'all'));
            point = states{find(present, 1)};
            testCase.verifyTrue(isfield(point, 'normalLoads_N'));
            testCase.verifyTrue(isfield(point, 'slipAngles_rad'));
            testCase.verifyTrue(isfield(point, 'slipRatios'));
        end

        function comparisonContainsSegmentTimeLoss(testCase)
            reference.distance_m = [0; 1; 2];
            reference.speed_mps = [2; 2; 2];
            reference.time_s = [0; 0.5; 1];
            candidate = reference;
            candidate.speed_mps = [1; 1; 1];
            candidate.time_s = [0; 1; 2];
            comparison = lts.analysis.compareResults(reference, candidate);
            testCase.verifyEqual(comparison.segmentTimeDifference_s, ...
                [0.5; 0.5], AbsTol=1e-12);
            testCase.verifyEqual(comparison.cumulativeTimeDifference_s(end), ...
                comparison.lapTimeDifference_s, AbsTol=1e-12);
        end
    end
end
