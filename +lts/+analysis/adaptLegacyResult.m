function result = adaptLegacyResult(distance_m, speed_mps, curvature_1pm, time_s)
%ADAPTLEGACYRESULT Adapt exported legacy profile arrays to V2 result shape.
arguments
    distance_m (:, 1) double {mustBeFinite}
    speed_mps (:, 1) double {mustBeFinite, mustBeNonnegative}
    curvature_1pm (:, 1) double {mustBeFinite}
    time_s (:, 1) double {mustBeFinite, mustBeNonnegative}
end
assert(numel(distance_m) == numel(speed_mps) && ...
    numel(distance_m) == numel(curvature_1pm) && ...
    numel(distance_m) == numel(time_s), 'lts:analysis:LegacyShape', ...
    'Legacy profile arrays must have equal length.');
assert(all(diff(distance_m) > 0) && all(diff(time_s) >= 0), ...
    'lts:analysis:LegacyOrder', ...
    'Distance must increase and time must not decrease.');
result.meta.modelName = 'Legacy exported profile';
result.meta.strategy = 'legacyExternal';
result.meta.status = 'adapted';
result.lap.distance_m = distance_m;
result.lap.speed_mps = speed_mps;
result.lap.curvature_1pm = curvature_1pm;
result.lap.time_s = time_s;
result.lap.lapTime_s = time_s(end);
result.lap.lateralAcceleration_mps2 = speed_mps.^2 .* curvature_1pm;
result.lap.segmentAcceleration_mps2 = diff(speed_mps.^2) ./ (2 .* diff(distance_m));
end
