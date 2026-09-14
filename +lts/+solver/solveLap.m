function result = solveLap(track, model, options)
%SOLVELAP Compute a QSS speed profile from a vehicle performance interface.
% The solver knows only curvature and the model's acceleration limits.

arguments
    track (1, 1) struct
    model (1, 1) struct
    options.initialSpeed_mps (1, 1) double {mustBeNonnegative} = 0
    options.finalSpeed_mps (1, 1) double {mustBeNonnegative} = Inf
    options.maximumSpeed_mps (1, 1) double {mustBePositive} = 100
    options.maximumIterations (1, 1) double {mustBeInteger, mustBePositive} = 200
    options.speedTolerance_mps (1, 1) double {mustBePositive} = 1e-8
end

lts.validation.validateTrack(track);
requiredMethods = {'maximumSpeedForCurvature', 'accelerationLimits'};
for index = 1:numel(requiredMethods)
    assert(isfield(model, requiredMethods{index}) ...
        && isa(model.(requiredMethods{index}), 'function_handle'), ...
        'lts:solver:ModelInterface', 'Missing model method: %s.', ...
        requiredMethods{index});
end

distance_m = track.distance_m;
curvature_1pm = track.curvature_1pm;
segmentLength_m = diff(distance_m);
speed_mps = min(model.maximumSpeedForCurvature(curvature_1pm), ...
    options.maximumSpeed_mps);

if track.isClosed
    speed_mps(end) = min(speed_mps(end), speed_mps(1));
    speed_mps(1) = speed_mps(end);
else
    speed_mps(1) = min(speed_mps(1), options.initialSpeed_mps);
    speed_mps(end) = min(speed_mps(end), options.finalSpeed_mps);
end

converged = false;
for iteration = 1:options.maximumIterations
    previousSpeed_mps = speed_mps;

    for point = 2:numel(speed_mps)
        lateralAcceleration_mps2 = speed_mps(point - 1)^2 ...
            * curvature_1pm(point - 1);
        limits = model.accelerationLimits(speed_mps(point - 1), ...
            lateralAcceleration_mps2);
        reachableSpeed_mps = sqrt(max(0, speed_mps(point - 1)^2 ...
            + 2 * limits.maximumAcceleration_mps2 * segmentLength_m(point - 1)));
        speed_mps(point) = min(speed_mps(point), reachableSpeed_mps);
    end

    for point = numel(speed_mps)-1:-1:1
        lateralAcceleration_mps2 = speed_mps(point + 1)^2 ...
            * curvature_1pm(point + 1);
        limits = model.accelerationLimits(speed_mps(point + 1), ...
            lateralAcceleration_mps2);
        reachableSpeed_mps = sqrt(max(0, speed_mps(point + 1)^2 ...
            + 2 * limits.maximumDeceleration_mps2 * segmentLength_m(point)));
        speed_mps(point) = min(speed_mps(point), reachableSpeed_mps);
    end

    if track.isClosed
        boundarySpeed_mps = min(speed_mps([1, end]));
        speed_mps([1, end]) = boundarySpeed_mps;
    end
    maximumChange_mps = max(abs(speed_mps - previousSpeed_mps));
    if maximumChange_mps <= options.speedTolerance_mps
        converged = true;
        break
    end
end

assert(converged, 'lts:solver:NoConvergence', ...
    'Lap solver did not converge within %d iterations.', options.maximumIterations);
segmentTime_s = 2 * segmentLength_m ./ ...
    (speed_mps(1:end-1) + speed_mps(2:end));
assert(all(isfinite(segmentTime_s)), 'lts:solver:InfiniteLapTime', ...
    'A segment has zero average speed and therefore infinite travel time.');

time_s = [0; cumsum(segmentTime_s)];
lateralAcceleration_mps2 = speed_mps.^2 .* curvature_1pm;
segmentAcceleration_mps2 = diff(speed_mps.^2) ./ (2 * segmentLength_m);

result.meta.modelName = model.name;
result.meta.modelLevel = model.level;
result.meta.iterations = iteration;
result.meta.converged = converged;
result.lapTime_s = time_s(end);
result.distance_m = distance_m;
result.time_s = time_s;
result.speed_mps = speed_mps;
result.curvature_1pm = curvature_1pm;
result.lateralAcceleration_mps2 = lateralAcceleration_mps2;
result.segmentAcceleration_mps2 = segmentAcceleration_mps2;
end
