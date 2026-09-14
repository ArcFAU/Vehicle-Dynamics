function comparison = compareResults(reference, candidate)
%COMPARERESULTS Compare V2/legacy-adapted lap profiles on common distance.
ref = getLap(reference);
got = getLap(candidate);
assert(isfield(ref, 'distance_m') && isfield(got, 'distance_m'), ...
    'lts:analysis:CompareShape', 'Both results require lap distance_m.');
distance = ref.distance_m(:);
candidateSpeed = interp1(got.distance_m(:), got.speed_mps(:), ...
    distance, 'linear', 'extrap');
speedDelta = candidateSpeed - ref.speed_mps(:);
segmentLength_m = diff(distance);
referenceSegmentTime_s = 2 .* segmentLength_m ./ ...
    (ref.speed_mps(1:end-1) + ref.speed_mps(2:end));
candidateSegmentTime_s = 2 .* segmentLength_m ./ ...
    (candidateSpeed(1:end-1) + candidateSpeed(2:end));
segmentTimeDifference_s = candidateSegmentTime_s - referenceSegmentTime_s;
comparison.referenceLapTime_s = ref.time_s(end);
comparison.candidateLapTime_s = got.time_s(end);
comparison.lapTimeDifference_s = comparison.candidateLapTime_s - ...
    comparison.referenceLapTime_s;
comparison.lapTimeDifference_percent = 100 * comparison.lapTimeDifference_s / ...
    comparison.referenceLapTime_s;
comparison.speedRmsDifference_mps = sqrt(mean(speedDelta.^2));
comparison.speedMaximumAbsoluteDifference_mps = max(abs(speedDelta));
comparison.distance_m = distance;
comparison.speedDifference_mps = speedDelta;
comparison.segmentMidpoint_m = ...
    (distance(1:end-1) + distance(2:end)) ./ 2;
comparison.segmentTimeDifference_s = segmentTimeDifference_s;
comparison.cumulativeTimeDifference_s = [0; cumsum(segmentTimeDifference_s)];
comparison.note = ['Diagnostic profile comparison; differences do not by ', ...
    'themselves identify a model error.'];
end

function lap = getLap(result)
if isfield(result, 'lap')
    lap = result.lap;
else
    lap = result;
end
end
