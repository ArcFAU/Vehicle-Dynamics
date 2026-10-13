function output = runAeroStateEnvelopeV2()
%RUNAEROSTATEENVELOPEV2 Derive quasi-static CFD vehicle states from a V2 lap.
%
% The result is an operating-point cloud. It is suitable for defining and
% prioritising a steady-state aerodynamic test matrix, but it does not
% represent transient kerb, bump, damper or platform dynamics.
% AeroYaw_deg is the incoming-flow angle and therefore uses the opposite
% sign of the vehicle body-slip angle at zero ambient wind.

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(projectRoot,fullfile(projectRoot,'vehicles'),fullfile(projectRoot,'config'));


%% Explicit aero-interface assumptions
settings.model = "dualTrack"; % "singleTrack" or "dualTrack"
settings.nominalFrontRideHeight_mm = 35;
settings.nominalRearRideHeight_mm = 35;
settings.rollCenterHeight_mm = 67.5;
settings.tireSocketHeight_mm = 1.0;
settings.tireSquishDepth_mm = NaN; % Requires a validated load-deflection law.
settings.windYawOffset_deg = 0;    % Positive value adds crosswind incidence.
settings.percentiles = [0.5, 5, 50, 95, 99.5];
settings.writeFiles = true;

%% Independent V2 lap case
input.vehicle = FAUmaxTauV2("fourMotorAWD");
tracks = lts.io.trackCatalog();
input.track = lts.io.loadTrackMatrix(tracks.endurance, true);

assert(any(settings.model == ["singleTrack", "dualTrack"]), ...
    'settings.model must be "singleTrack" or "dualTrack".');
input.model = settings.model;
input.features.aero = true;
input.features.loadTransfer = true;
input.features.tireSlip = false;
input.features.energy = false;
input.features.yawInertia = false;
input.features.chassisTorsion = false;

input.solver.strategy = "direct";
input.solver.maximumSpeed_mps = 120/3.6;
input.solver.initialSpeed_mps = 0;
input.solver.finalSpeed_mps = Inf;
input.solver.performanceMap = [];
input.solver.trackLocal.speedRows = 3;
input.solver.trackLocal.lateralFractions = [];
input.solver.trackLocal.boundaryIterations = 6;
input.solver.trackLocal.interpolationSafetyFactor = NaN;
input.solver.trackLocal.showProgress = false;
input.solver.trackLocal.auditPointCount = 8;

config = lts.v2.createCase(input);
lapResult = lts.v2.runCase(config);
vehicle = config.effectiveVehicle;
lap = lapResult.lap;

%% Reconstruct one quasi-static vehicle state per lap segment
segmentCount = numel(lap.segmentAcceleration_mps2);
selectedSegments = (1:segmentCount).';
speed_mps = (lap.speed_mps(1:end-1) + lap.speed_mps(2:end)) / 2;
curvature_1pm = (lap.curvature_1pm(1:end-1) ...
    + lap.curvature_1pm(2:end)) / 2;
lateralAcceleration_mps2 = speed_mps .^ 2 .* curvature_1pm;
longitudinalAcceleration_mps2 = lap.segmentAcceleration_mps2;
yawDemand = lts.analysis.yawDemand(lap, vehicle);
targetYawMoment_Nm = 0.5 * (yawDemand.inertialMoment_Nm(1:end-1) ...
    + yawDemand.inertialMoment_Nm(2:end));

state = lts.model.dualTrackEnvelope.calculateState(vehicle, speed_mps, ...
    longitudinalAcceleration_mps2, lateralAcceleration_mps2);
normalLoads_N = state.normalLoads_N;
yawRate_degps = rad2deg(speed_mps .* curvature_1pm);

%% Recover sideslip and steering from the stationary tire-slip model
bodySlip_deg = nan(segmentCount, 1);
steer_deg = nan(segmentCount, 1);
slipSolverValid = false(segmentCount, 1);
slipSolverExitFlag = nan(segmentCount, 1);
slipSolverResidualInf = nan(segmentCount, 1);
slipSolverMessage = strings(segmentCount, 1);
slipAngles_deg = nan(segmentCount, 4);
slipRatios = nan(segmentCount, 4);
withinSteeringLimits = false(segmentCount, 1);
withinSlipAngleLimits = false(segmentCount, 1);
withinSlipRatioLimits = false(segmentCount, 1);
withinNormalLoadLimits = false(segmentCount, 1);
awayFromSolverBounds = false(segmentCount, 1);
operatingPointPhysicallyValid = false(segmentCount, 1);
initialUnknowns = [];
for index = 1:segmentCount
    try
        point = solveSlipOperatingPoint(settings.model, vehicle, ...
            speed_mps(index), longitudinalAcceleration_mps2(index), ...
            lateralAcceleration_mps2(index), targetYawMoment_Nm(index), ...
            initialUnknowns);
        bodySlip_deg(index) = rad2deg(point.bodySlipAngle_rad);
        steer_deg(index) = rad2deg(point.steeringAngle_rad);
        slipSolverValid(index) = point.feasible;
        slipSolverExitFlag(index) = point.solver.exitFlag;
        slipSolverResidualInf(index) = ...
            norm(point.solver.normalizedResidual, Inf);
        slipAngles_deg(index, :) = rad2deg(point.slipAngles_rad);
        slipRatios(index, :) = point.slipRatios;
        withinSteeringLimits(index) = ...
            point.validity.withinSteeringLimits;
        withinSlipAngleLimits(index) = ...
            point.validity.withinSlipAngleLimits;
        withinSlipRatioLimits(index) = ...
            point.validity.withinSlipRatioLimits;
        withinNormalLoadLimits(index) = ...
            point.validity.withinNormalLoadLimits;
        awayFromSolverBounds(index) = ...
            point.validity.awayFromSolverBounds;
        operatingPointPhysicallyValid(index) = point.physicallyValid;
        if point.physicallyValid
            initialUnknowns = point.solver.solution;
            slipSolverMessage(index) = "valid";
        else
            initialUnknowns = [];
            slipSolverMessage(index) = validityMessage(point);
        end
    catch exception
        initialUnknowns = [];
        slipSolverMessage(index) = string(exception.identifier) ...
            + ": " + string(exception.message);
    end
end
aeroYaw_deg = -bodySlip_deg + settings.windYawOffset_deg;

%% Derive quasi-static platform position
roll = lts.suspension.rollProperties(vehicle);
mass_kg = vehicle.mass.total_kg;
frontFraction = vehicle.geometry.frontMassFraction;
staticFrontAxleLoad_N = mass_kg * 9.81 * frontFraction;
staticRearAxleLoad_N = mass_kg * 9.81 * (1 - frontFraction);

frontAxleLoad_N = sum(normalLoads_N(:, 1:2), 2);
rearAxleLoad_N = sum(normalLoads_N(:, 3:4), 2);
frontCompression_m = (frontAxleLoad_N - staticFrontAxleLoad_N) ...
    / (2 * roll.frontWheelRate_Npm);
rearCompression_m = (rearAxleLoad_N - staticRearAxleLoad_N) ...
    / (2 * roll.rearWheelRate_Npm);

frontRideHeight_mm = settings.nominalFrontRideHeight_mm ...
    - 1e3 * frontCompression_m;
rearRideHeight_mm = settings.nominalRearRideHeight_mm ...
    - 1e3 * rearCompression_m;
pitch_deg = rad2deg(atan2((rearRideHeight_mm - frontRideHeight_mm) / 1e3, ...
    vehicle.geometry.wheelbase_m));

rollMoment_Nm = mass_kg * lateralAcceleration_mps2 ...
    * vehicle.geometry.cgHeight_m;
rollAngle_deg = rad2deg(rollMoment_Nm / roll.totalRollStiffness_NmPerRad);

cgPositionFromFrontFraction = 1 - frontFraction;
referenceRideHeight_mm = frontRideHeight_mm ...
    + cgPositionFromFrontFraction ...
    .* (rearRideHeight_mm - frontRideHeight_mm);

rollAngle_rad = deg2rad(rollAngle_deg);
rideHeightFL_mm = frontRideHeight_mm + 1e3 ...
    * vehicle.geometry.frontTrack_m / 2 .* tan(rollAngle_rad);
rideHeightFR_mm = frontRideHeight_mm - 1e3 ...
    * vehicle.geometry.frontTrack_m / 2 .* tan(rollAngle_rad);
rideHeightRL_mm = rearRideHeight_mm + 1e3 ...
    * vehicle.geometry.rearTrack_m / 2 .* tan(rollAngle_rad);
rideHeightRR_mm = rearRideHeight_mm - 1e3 ...
    * vehicle.geometry.rearTrack_m / 2 .* tan(rollAngle_rad);
cornerRideHeights_mm = [rideHeightFL_mm, rideHeightFR_mm, ...
    rideHeightRL_mm, rideHeightRR_mm];
withinRideHeightLimits = all(cornerRideHeights_mm ...
    >= vehicle.validity.minimumRideHeight_mm ...
    & cornerRideHeights_mm <= vehicle.validity.maximumRideHeight_mm, 2);

segmentMidpointTime_s = 0.5 * (lap.time_s(1:end-1) + lap.time_s(2:end));
steeringRate_degps = gradient(steer_deg, segmentMidpointTime_s);
bodySlipRate_degps = gradient(bodySlip_deg, segmentMidpointTime_s);
physicallyValid = operatingPointPhysicallyValid & withinRideHeightLimits;
rideHeightRejected = operatingPointPhysicallyValid & ~withinRideHeightLimits;
slipSolverMessage(rideHeightRejected) = ...
    slipSolverMessage(rideHeightRejected) + "; ride height outside limits";

%% CFD-oriented state cloud
states = table(lap.distance_m(selectedSegments), ...
    lap.time_s(selectedSegments), speed_mps, ...
    bodySlip_deg, aeroYaw_deg, steer_deg, ...
    rollAngle_deg, pitch_deg, frontRideHeight_mm, rearRideHeight_mm, ...
    referenceRideHeight_mm, yawRate_degps, ...
    longitudinalAcceleration_mps2, lateralAcceleration_mps2, ...
    normalLoads_N(:, 1), normalLoads_N(:, 2), normalLoads_N(:, 3), ...
    normalLoads_N(:, 4), targetYawMoment_Nm, ...
    slipAngles_deg(:, 1), slipAngles_deg(:, 2), ...
    slipAngles_deg(:, 3), slipAngles_deg(:, 4), ...
    slipRatios(:, 1), slipRatios(:, 2), slipRatios(:, 3), ...
    slipRatios(:, 4), rideHeightFL_mm, rideHeightFR_mm, ...
    rideHeightRL_mm, rideHeightRR_mm, steeringRate_degps, ...
    bodySlipRate_degps, slipSolverValid, withinSteeringLimits, ...
    withinSlipAngleLimits, withinSlipRatioLimits, ...
    withinNormalLoadLimits, awayFromSolverBounds, ...
    withinRideHeightLimits, physicallyValid, slipSolverExitFlag, ...
    slipSolverResidualInf, slipSolverMessage, ...
    'VariableNames', {'Distance_m', 'Time_s', 'Velocity_mps', ...
    'BodySlip_deg', 'AeroYaw_deg', 'SteerAngle_deg', ...
    'RollAngle_deg', 'Pitch_deg', ...
    'FRH_mm', 'RRH_mm', 'RH_mm', 'YawRate_degps', ...
    'LongitudinalAcceleration_mps2', 'LateralAcceleration_mps2', ...
    'NormalLoadFL_N', 'NormalLoadFR_N', 'NormalLoadRL_N', ...
    'NormalLoadRR_N', 'TargetYawMoment_Nm', ...
    'SlipAngleFL_deg', 'SlipAngleFR_deg', 'SlipAngleRL_deg', ...
    'SlipAngleRR_deg', 'SlipRatioFL', 'SlipRatioFR', 'SlipRatioRL', ...
    'SlipRatioRR', 'RideHeightFL_mm', 'RideHeightFR_mm', ...
    'RideHeightRL_mm', 'RideHeightRR_mm', 'SteeringRate_degps', ...
    'BodySlipRate_degps', 'SlipSolverValid', 'WithinSteeringLimits', ...
    'WithinSlipAngleLimits', 'WithinSlipRatioLimits', ...
    'WithinNormalLoadLimits', 'AwayFromSolverBounds', ...
    'WithinRideHeightLimits', 'PhysicallyValid', 'SlipSolverExitFlag', ...
    'SlipSolverResidualInf', 'RejectionReason'});

validStates = states(states.PhysicallyValid, :);
assert(~isempty(validStates), ...
    'No valid tire-slip operating point is available for CFD export.');
stateNames = {'Velocity_mps', 'BodySlip_deg', 'AeroYaw_deg', ...
    'SteerAngle_deg', ...
    'RollAngle_deg', 'Pitch_deg', 'FRH_mm', 'RRH_mm', 'RH_mm'};
summary = createSummary(validStates, stateNames, settings.percentiles);

metadata.notes = {
    'BodySlip_deg is atan2(vehicle lateral velocity, longitudinal velocity).'
    ['AeroYaw_deg = -BodySlip_deg + windYawOffset_deg and describes the ', ...
    'incoming airflow angle in the convention used by this export.']
    'SteerAngle_deg is recovered by the stationary tire-slip model.'
    'YawRate_degps is reported separately and is not aerodynamic yaw.'
    ['Only rows with PhysicallyValid=true are included in cfdStates and ', ...
    'the percentile summary.']
    'TargetYawMoment_Nm includes the quasi-transient inertial yaw demand.'
    ['SteeringRate_degps and BodySlipRate_degps are diagnostic only until ', ...
    'validated actuator and vehicle-rate limits are available.']
    'Ride heights and pitch are quasi-static spring-deflection estimates.'
    'Roll follows the load-transfer roll moment used by the V2 model.'
    'Tire squish is unavailable without a validated load-deflection law.'
    'Transient kerb, bump and damper motion is outside this QSS result.'};
metadata.wheelbase_mm = 1e3 * vehicle.geometry.wheelbase_m;
metadata.rollCenterHeight_mm = settings.rollCenterHeight_mm;
metadata.tireSocketHeight_mm = settings.tireSocketHeight_mm;
metadata.tireSquishDepth_mm = settings.tireSquishDepth_mm;
metadata.track = tracks.endurance;
metadata.lapModel = settings.model;
metadata.slipOperatingPointModel = settings.model + "Slip";

output.settings = settings;
output.states = states;
output.cfdStates = validStates;
output.summary = summary;
output.metadata = metadata;
output.lapResult = lapResult;

if settings.writeFiles
    output.files = writeOutputs(projectRoot, output);
end

disp(summary);
fprintf('Lap time: %.3f s\n', lap.lapTime_s);
fprintf('Valid CFD states: %d of %d\n', height(validStates), height(states));
if isfield(output, 'files')
    fprintf('State cloud: %s\n', output.files.csv);
    fprintf('Valid CFD states: %s\n', output.files.cfdCsv);
    fprintf('Complete result: %s\n', output.files.mat);
end
end

function summary = createSummary(states, stateNames, requestedPercentiles)
rowNames = ["Minimum", "P" + replace(string(requestedPercentiles), ".", "_"), ...
    "Maximum"];
values = zeros(numel(rowNames), numel(stateNames));
for column = 1:numel(stateNames)
    data = states.(stateNames{column});
    values(:, column) = [min(data); ...
        prctile(data, requestedPercentiles(:)); max(data)];
end
summary = array2table(values, 'VariableNames', stateNames, ...
    'RowNames', cellstr(rowNames));
end

function point = solveSlipOperatingPoint(model, vehicle, speed_mps, ...
    longitudinalAcceleration_mps2, lateralAcceleration_mps2, ...
    targetYawMoment_Nm, initialUnknowns)
if model == "singleTrack"
    point = lts.model.singleTrackSlip.operatingPoint(vehicle, speed_mps, ...
        longitudinalAcceleration_mps2, lateralAcceleration_mps2, ...
        targetYawMoment_Nm=targetYawMoment_Nm, enableRetries=true, ...
        initialUnknowns=initialUnknowns);
else
    point = lts.model.dualTrackSlip.operatingPoint(vehicle, speed_mps, ...
        longitudinalAcceleration_mps2, lateralAcceleration_mps2, ...
        targetYawMoment_Nm=targetYawMoment_Nm, enableRetries=true, ...
        initialUnknowns=initialUnknowns);
end
end

function message = validityMessage(point)
reasons = strings(0, 1);
if ~point.feasible
    reasons(end + 1) = "solver result not feasible";
end
names = { ...
    'withinSteeringLimits', "steering outside limits"; ...
    'withinSlipAngleLimits', "slip angle outside limits"; ...
    'withinSlipRatioLimits', "slip ratio outside limits"; ...
    'withinNormalLoadLimits', "normal load outside limits"; ...
    'awayFromSolverBounds', "solution too close to solver bounds"};
for index = 1:size(names, 1)
    if ~point.validity.(names{index, 1})
        reasons(end + 1) = names{index, 2}; %#ok<AGROW>
    end
end
if isempty(reasons)
    message = "unknown validity failure";
else
    message = join(reasons, "; ");
end
end

function files = writeOutputs(projectRoot, output)
resultsFolder = fullfile(projectRoot, 'results');
if ~isfolder(resultsFolder)
    mkdir(resultsFolder);
end
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
baseName = ['aeroStateEnvelopeV2_', char(output.settings.model), '_', timestamp];
files.csv = fullfile(resultsFolder, [baseName, '.csv']);
files.cfdCsv = fullfile(resultsFolder, [baseName, '_cfd-valid.csv']);
files.summaryCsv = fullfile(resultsFolder, [baseName, '_summary.csv']);
files.mat = fullfile(resultsFolder, [baseName, '.mat']);
writetable(output.states, files.csv);
writetable(output.cfdStates, files.cfdCsv);
writetable(output.summary, files.summaryCsv, 'WriteRowNames', true);
save(files.mat, 'output');
end
