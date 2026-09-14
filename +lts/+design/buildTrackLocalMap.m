function [map, info] = buildTrackLocalMap(vehicle, track, options)
%BUILDTRACKLOCALMAP Build a compact slip envelope for one concrete track.
% The same nonlinear operating-point solver as the global performance map
% is used, but only over the speed range relevant to the current track.

arguments
    vehicle (1, 1) struct
    track (1, 1) struct
    options.Model (1, 1) string
    options.MaximumSpeed_mps (1, 1) double {mustBePositive} = 100
    options.InitialSpeed_mps (1, 1) double {mustBeNonnegative} = 0
    options.FinalSpeed_mps (1, 1) double {mustBeNonnegative} = Inf
    options.SpeedRowCount (1, 1) double ...
        {mustBeInteger, mustBeGreaterThanOrEqual(options.SpeedRowCount, 2)} = 3
    options.LateralFractions (1, :) double = []
    options.BoundaryIterations (1, 1) double ...
        {mustBeInteger, mustBePositive} = 6
    options.InterpolationSafetyFactor (1, 1) double = NaN
    options.ShowProgress (1, 1) logical = false
end

lts.validation.validateVehicle(vehicle);
lts.validation.validateTrack(track);
definition = lts.model.registry(options.Model);
assert(any(definition.strategies == "trackLocal"), ...
    'lts:design:TrackLocalModel', ...
    'Track-local execution is unavailable for model "%s".', options.Model);
switch definition.mapType
    case "singleTrack"
        seedModel = lts.model.singleTrackAero.create(vehicle);
        defaultSafetyFactor = 1.00;
    case "dualTrack"
        seedModel = lts.model.dualTrackEnvelope.create(vehicle);
        defaultSafetyFactor = 1.00;
    otherwise
        error('lts:design:TrackLocalModel', ...
            'Model "%s" has no track-local map type.', options.Model);
end

seedLap = lts.solver.solveLap(track, seedModel, ...
    initialSpeed_mps=options.InitialSpeed_mps, ...
    finalSpeed_mps=options.FinalSpeed_mps, ...
    maximumSpeed_mps=options.MaximumSpeed_mps);
positiveSpeeds_mps = seedLap.speed_mps(seedLap.speed_mps > 0);
assert(~isempty(positiveSpeeds_mps), 'lts:design:TrackLocalSeed', ...
    'The seed lap contains no positive speed.');

drive = lts.powertrain.maximumDriveForce(vehicle.powertrain, 0);
lowerSpeed_mps = min(3, max(0.5, min(positiveSpeeds_mps)));
upperSpeed_mps = min([options.MaximumSpeed_mps, ...
    drive.maximumVehicleSpeed_mps, 1.03 * max(positiveSpeeds_mps)]);
if upperSpeed_mps <= lowerSpeed_mps
    lowerSpeed_mps = max(0.1, 0.5 * upperSpeed_mps);
end
speedGrid_mps = representativeValues(positiveSpeeds_mps, ...
    options.SpeedRowCount, lowerSpeed_mps, upperSpeed_mps);

lateralFractions = options.LateralFractions;
if isempty(lateralFractions)
    absoluteLateralAcceleration_mps2 = ...
        abs(seedLap.lateralAcceleration_mps2);
    maximumLateralAcceleration_mps2 = ...
        max(absoluteLateralAcceleration_mps2);
    if maximumLateralAcceleration_mps2 > 0
        seedFractions = absoluteLateralAcceleration_mps2 ...
            / maximumLateralAcceleration_mps2;
        lateralFractions = representativeValues(seedFractions, 3, 0, 1);
    else
        lateralFractions = [0, 0.5, 1];
    end
end
validateattributes(lateralFractions, {'numeric'}, ...
    {'row', 'real', 'finite', 'nonnegative', '<=', 1, 'increasing'});
assert(lateralFractions(1) == 0 && lateralFractions(end) == 1, ...
    'lts:design:TrackLocalFractions', ...
    'Track-local lateral fractions must start at zero and end at one.');

safetyFactor = options.InterpolationSafetyFactor;
if isnan(safetyFactor)
    safetyFactor = defaultSafetyFactor;
end
validateattributes(safetyFactor, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'positive', '<=', 1});

buildTimer = tic;
switch definition.mapType
    case "singleTrack"
        map = lts.model.singleTrackSlip.buildPerformanceMap(vehicle, ...
            speedGrid_mps=speedGrid_mps, ...
            lateralFractions=lateralFractions, ...
            boundaryIterations=options.BoundaryIterations, ...
            interpolationSafetyFactor=safetyFactor, ...
            showProgress=options.ShowProgress);
    otherwise
        map = lts.model.dualTrackSlip.buildPerformanceMap(vehicle, ...
            speedGrid_mps=speedGrid_mps, ...
            lateralFractions=lateralFractions, ...
            boundaryIterations=options.BoundaryIterations, ...
            interpolationSafetyFactor=safetyFactor, ...
            showProgress=options.ShowProgress, ...
            progressLabel="Track-local slip envelope", ...
            enableSolverRetries=true);
end
buildTime_s = toc(buildTimer);

map.meta.scope = 'trackLocal';
map.meta.trackName = trackName(track);
map.meta.trackLength_m = track.distance_m(end) - track.distance_m(1);
map.meta.trackMaximumAbsoluteCurvature_1pm = ...
    max(abs(track.curvature_1pm));
map.meta.gridSelection = 'seedLapQuantiles';

info.scope = 'trackLocal';
info.modelId = char(definition.id);
info.trackName = map.meta.trackName;
info.seedModel = seedModel.name;
info.seedLapTime_s = seedLap.lapTime_s;
info.buildTime_s = buildTime_s;
info.speedRange_mps = speedGrid_mps([1, end]);
info.speedRowCount = numel(speedGrid_mps);
info.lateralFractions = lateralFractions;
info.boundaryIterations = options.BoundaryIterations;
info.interpolationSafetyFactor = safetyFactor;
info.boundariesCalculated = numel(speedGrid_mps) * 2 ...
    * (1 + 2 * numel(lateralFractions));
info.gridSelection = map.meta.gridSelection;
end

function name = trackName(track)
name = 'unnamed track';
if isfield(track, 'meta') && isfield(track.meta, 'name')
    name = track.meta.name;
end
end

function values = representativeValues(samples, count, lower, upper)
samples = sort(min(max(samples(:), lower), upper));
samples = [lower; samples; upper];
indices = round(linspace(1, numel(samples), count));
values = unique(samples(indices), 'sorted').';
if numel(values) ~= count || any(diff(values) <= 0)
    values = linspace(lower, upper, count);
end
values(1) = lower;
values(end) = upper;
end
