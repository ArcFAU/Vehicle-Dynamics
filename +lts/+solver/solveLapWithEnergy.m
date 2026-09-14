function result = solveLapWithEnergy(track, model, options)
%SOLVELAPWITHENERGY Couple SoC-dependent drive power into the speed profile.
% Total braking remains available through regeneration plus friction brakes.

arguments
    track (1, 1) struct
    model (1, 1) struct
    options.Repetitions (1, 1) double {mustBeInteger, mustBePositive} = 1
    options.InitialSoc (1, 1) double {mustBeInRange(options.InitialSoc, 0, 1)} = 0.95
    options.InitialSpeed_mps (1, 1) double = NaN
    options.FinalSpeed_mps (1, 1) double {mustBeNonnegative} = 100
    options.MaximumSpeed_mps (1, 1) double {mustBePositive} = 100
    options.MaximumIterations (1, 1) double {mustBeInteger, mustBePositive} = 30
    options.SpeedTolerance_mps (1, 1) double {mustBePositive} = 1e-5
end

lts.validation.validateTrack(track);
assert(isfield(model, 'vehicle'), 'lts:solver:EnergyVehicle', ...
    'The energy-coupled solver requires model.vehicle.');
vehicle = model.vehicle;
vehicle.energy.initialSoc = options.InitialSoc;
expandedTrack = expandTrack(track, options.Repetitions);
baseLap = lts.solver.solveLap(track, model, ...
    initialSpeed_mps=defaultInitialSpeed(track, options.InitialSpeed_mps), ...
    finalSpeed_mps=options.FinalSpeed_mps, ...
    maximumSpeed_mps=options.MaximumSpeed_mps);
speed_mps = expandSpeed(baseLap.speed_mps, options.Repetitions);
distance_m = expandedTrack.distance_m;
curvature_1pm = expandedTrack.curvature_1pm;
segmentLength_m = diff(distance_m);
initialSpeed_mps = speed_mps(1);
if ~isnan(options.InitialSpeed_mps)
    initialSpeed_mps = options.InitialSpeed_mps;
end

converged = false;
for iteration = 1:options.MaximumIterations
    previousSpeed_mps = speed_mps;
    speed_mps = backwardPass(speed_mps, curvature_1pm, ...
        segmentLength_m, model);
    [speed_mps, socBoundary] = forwardEnergyPass(speed_mps, ...
        curvature_1pm, segmentLength_m, model, vehicle, ...
        initialSpeed_mps, options.InitialSoc);
    if max(abs(speed_mps - previousSpeed_mps)) ...
            <= options.SpeedTolerance_mps
        converged = true;
        break
    end
end
assert(converged, 'lts:solver:EnergyNoConvergence', ...
    'Energy-coupled solver did not converge within %d iterations.', ...
    options.MaximumIterations);

segmentTime_s = 2 * segmentLength_m ...
    ./ (speed_mps(1:end-1) + speed_mps(2:end));
assert(all(isfinite(segmentTime_s)), 'lts:solver:InfiniteLapTime', ...
    'The energy-coupled profile contains a zero-speed segment.');
time_s = [0; cumsum(segmentTime_s)];
result.meta.modelName = model.name;
result.meta.modelLevel = '2.4c';
result.meta.baseModelLevel = model.level;
result.meta.iterations = iteration;
result.meta.converged = converged;
result.meta.repetitions = options.Repetitions;
result.lapTime_s = time_s(end);
result.distance_m = distance_m;
result.time_s = time_s;
result.speed_mps = speed_mps;
result.curvature_1pm = curvature_1pm;
result.lateralAcceleration_mps2 = speed_mps .^ 2 .* curvature_1pm;
result.segmentAcceleration_mps2 = ...
    diff(speed_mps .^ 2) ./ (2 * segmentLength_m);
result.socBoundary = socBoundary;
result.energy = lts.energy.analyzeLap(result, vehicle);
end

function speed_mps = backwardPass(speed_mps, curvature_1pm, ...
        segmentLength_m, model)
for point = numel(speed_mps)-1:-1:1
    lateralAcceleration_mps2 = speed_mps(point + 1) ^ 2 ...
        * curvature_1pm(point + 1);
    limits = model.accelerationLimits(speed_mps(point + 1), ...
        lateralAcceleration_mps2);
    reachableSpeed_mps = sqrt(max(0, speed_mps(point + 1) ^ 2 ...
        + 2 * limits.maximumDeceleration_mps2 * segmentLength_m(point)));
    speed_mps(point) = min(speed_mps(point), reachableSpeed_mps);
end
end

function [speed_mps, socBoundary] = forwardEnergyPass(speed_mps, ...
        curvature_1pm, segmentLength_m, model, vehicle, ...
        initialSpeed_mps, initialSoc)
speed_mps(1) = min(speed_mps(1), initialSpeed_mps);
socBoundary = zeros(numel(speed_mps), 1);
socBoundary(1) = initialSoc;
for point = 2:numel(speed_mps)
    speedBefore_mps = speed_mps(point - 1);
    lateralAcceleration_mps2 = speedBefore_mps ^ 2 ...
        * curvature_1pm(point - 1);
    limits = model.accelerationLimits(speedBefore_mps, ...
        lateralAcceleration_mps2);
    maximumAcceleration_mps2 = energyLimitedAcceleration(vehicle, ...
        speedBefore_mps, socBoundary(point - 1), ...
        limits.maximumAcceleration_mps2);
    reachableSpeed_mps = sqrt(max(0, speedBefore_mps ^ 2 ...
        + 2 * maximumAcceleration_mps2 * segmentLength_m(point - 1)));
    speed_mps(point) = min(speed_mps(point), reachableSpeed_mps);
    averageSpeed_mps = (speedBefore_mps + speed_mps(point)) / 2;
    segmentTime_s = 2 * segmentLength_m(point - 1) ...
        / (speedBefore_mps + speed_mps(point));
    actualAcceleration_mps2 = (speed_mps(point) ^ 2 ...
        - speedBefore_mps ^ 2) / (2 * segmentLength_m(point - 1));
    wheelForce_N = requiredWheelForce(vehicle, averageSpeed_mps, ...
        actualAcceleration_mps2);
    step = lts.energy.propagateSegment(vehicle, ...
        socBoundary(point - 1), averageSpeed_mps, wheelForce_N, ...
        segmentLength_m(point - 1), segmentTime_s);
    socBoundary(point) = step.nextSoc;
end
end

function maximumAcceleration_mps2 = energyLimitedAcceleration(vehicle, ...
        speed_mps, soc, modelMaximumAcceleration_mps2)
if speed_mps <= 0.5
    maximumAcceleration_mps2 = modelMaximumAcceleration_mps2;
    return
end
limits = lts.energy.limitsAtSoc(vehicle, soc);
baseWheelForce_N = requiredWheelForce(vehicle, speed_mps, ...
    modelMaximumAcceleration_mps2);
efficiency = lts.energy.conversionEfficiencies(vehicle, speed_mps, ...
    baseWheelForce_N, soc, "drive");
availableTractionBatteryPower_W = max(0, ...
    limits.maximumBatteryDischargePower_W ...
    - vehicle.energy.auxiliaryPower_W);
maximumWheelForce_N = availableTractionBatteryPower_W ...
    * efficiency.total / speed_mps;
roadLoad_N = requiredWheelForce(vehicle, speed_mps, 0);
powerLimitedAcceleration_mps2 = ...
    (maximumWheelForce_N - roadLoad_N) / vehicle.mass.total_kg;
maximumAcceleration_mps2 = min(modelMaximumAcceleration_mps2, ...
    max(0, powerLimitedAcceleration_mps2));
end

function wheelForce_N = requiredWheelForce(vehicle, speed_mps, ...
        acceleration_mps2)
aero = lts.aerodynamics.forces(vehicle.aerodynamics, speed_mps);
rollingForce_N = vehicle.energy.rollingResistanceCoefficient ...
    * (vehicle.mass.total_kg * 9.81 + aero.totalDownforce_N);
wheelForce_N = vehicle.mass.total_kg * acceleration_mps2 ...
    + aero.drag_N + rollingForce_N;
end

function expanded = expandTrack(track, repetitions)
if repetitions > 1
    assert(track.isClosed, 'lts:solver:EnergyRepetitions', ...
        'Repeated energy-coupled tracks must be closed.');
end
baseSegmentLength_m = diff(track.distance_m(:));
baseCurvature_1pm = track.curvature_1pm(:);
expanded.distance_m = [0; cumsum(repmat(baseSegmentLength_m, repetitions, 1))];
expanded.curvature_1pm = [repmat(baseCurvature_1pm(1:end-1), ...
    repetitions, 1); baseCurvature_1pm(end)];
expanded.isClosed = false;
expanded.meta = track.meta;
expanded.meta.name = [track.meta.name, '_energyCoupled'];
end

function expandedSpeed_mps = expandSpeed(baseSpeed_mps, repetitions)
baseSpeed_mps = baseSpeed_mps(:);
expandedSpeed_mps = [repmat(baseSpeed_mps(1:end-1), repetitions, 1); ...
    baseSpeed_mps(end)];
end

function initialSpeed_mps = defaultInitialSpeed(track, requestedSpeed_mps)
if isnan(requestedSpeed_mps)
    if track.isClosed
        initialSpeed_mps = 100;
    else
        initialSpeed_mps = 0;
    end
else
    initialSpeed_mps = requestedSpeed_mps;
end
end
