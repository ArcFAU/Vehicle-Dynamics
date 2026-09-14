function map = buildPerformanceMap(vehicle, options)
%BUILDPERFORMANCEMAP Build Level 2.1c boundaries from nonlinear tire states.

arguments
    vehicle (1, 1) struct
    options.speedGrid_mps (1, :) double = []
    options.lateralFractions (1, :) double = linspace(0, 1, 7)
    options.boundaryIterations (1, 1) double ...
        {mustBeInteger, mustBePositive} = 12
    options.maximumAccelerationSearch_mps2 (1, 1) double ...
        {mustBePositive, mustBeFinite} = 5 * 9.81
    options.interpolationSafetyFactor (1, 1) double ...
        {mustBePositive, mustBeFinite} = 0.80
    options.showProgress (1, 1) logical = true
    options.progressLabel (1, 1) string = "Level 2.1c map"
    options.enableSolverRetries (1, 1) logical = false
end

lts.validation.validateVehicle(vehicle);
mustBeLessThanOrEqual(options.interpolationSafetyFactor, 1);
drive = lts.powertrain.maximumDriveForce(vehicle.powertrain, 0);
if isempty(options.speedGrid_mps)
    options.speedGrid_mps = linspace(3, drive.maximumVehicleSpeed_mps, 9);
end
validateattributes(options.speedGrid_mps, {'numeric'}, ...
    {'row', 'real', 'finite', 'positive', 'increasing'});
validateattributes(options.lateralFractions, {'numeric'}, ...
    {'row', 'real', 'finite', 'nonnegative', '<=', 1, 'increasing'});
assert(options.lateralFractions(1) == 0 ...
    && options.lateralFractions(end) == 1, ...
    'lts:model:MapFractions', ...
    'Lateral fractions must start at zero and end at one.');

speedGrid_mps = options.speedGrid_mps;
lateralFractions = options.lateralFractions;
turnDirections = [-1, 1];
speedCount = numel(speedGrid_mps);
fractionCount = numel(lateralFractions);
directionCount = numel(turnDirections);
maximumLateralAcceleration_mps2 = zeros(speedCount, directionCount);
maximumAcceleration_mps2 = zeros(speedCount, fractionCount, directionCount);
maximumDeceleration_mps2 = zeros(speedCount, fractionCount, directionCount);
lateralBoundaryStates = cell(speedCount, directionCount);
accelerationBoundaryStates = cell(speedCount, fractionCount, directionCount);
decelerationBoundaryStates = cell(speedCount, fractionCount, directionCount);

for speedIndex = 1:speedCount
    speed_mps = speedGrid_mps(speedIndex);
    for directionIndex = 1:directionCount
        direction = turnDirections(directionIndex);
        [lateralLimit_mps2, lateralState] = findBoundary(@(magnitude, initial) ...
            query(vehicle, speed_mps, 0, direction * magnitude, ...
            options.enableSolverRetries, initial), ...
            options.maximumAccelerationSearch_mps2, ...
            options.boundaryIterations);
        maximumLateralAcceleration_mps2(speedIndex, directionIndex) = ...
            lateralLimit_mps2;
        lateralBoundaryStates{speedIndex, directionIndex} = lateralState;
        for fractionIndex = 1:fractionCount
            lateralAcceleration_mps2 = direction * lateralLimit_mps2 ...
                * lateralFractions(fractionIndex);
            [maximumAcceleration_mps2(speedIndex, fractionIndex, directionIndex), ...
                accelerationBoundaryStates{speedIndex, fractionIndex, directionIndex}] ...
                = findBoundary(@(magnitude, initial) query(vehicle, speed_mps, ...
                magnitude, lateralAcceleration_mps2, ...
                options.enableSolverRetries, initial), ...
                options.maximumAccelerationSearch_mps2, ...
                options.boundaryIterations);
            [maximumDeceleration_mps2(speedIndex, fractionIndex, directionIndex), ...
                decelerationBoundaryStates{speedIndex, fractionIndex, directionIndex}] ...
                = findBoundary(@(magnitude, initial) query(vehicle, speed_mps, ...
                -magnitude, lateralAcceleration_mps2, ...
                options.enableSolverRetries, initial), ...
                options.maximumAccelerationSearch_mps2, ...
                options.boundaryIterations);
        end
    end
    if options.showProgress
        fprintf('%s: %d/%d speed rows complete\n', ...
            options.progressLabel, ...
            speedIndex, speedCount);
    end
end

selectedTireModel = lts.tire.create(vehicle);
if strcmp(selectedTireModel.type, 'fittedCombinedSlip')
    map.meta.modelLevel = '2.1c';
else
    map.meta.modelLevel = '2.5c';
end
map.meta.tireModel = selectedTireModel.type;
map.meta.sourceVehicleDefinition = vehicle;
map.meta.tireParameterSet = selectedTireModel.parameterSet;
map.meta.tireModelVersion = selectedTireModel.version;
map.meta.tireParameterVersion = selectedTireModel.parameterVersion;
map.meta.tireFit = selectedTireModel.parameterSet;
map.meta.powertrainLayout = vehicle.powertrain.layout;
map.meta.driveAllocationMode = vehicle.controls.driveAllocation.mode;
map.meta.brakeAllocationMode = vehicle.controls.brakeAllocation.mode;
map.meta.targetYawMoment_Nm = vehicle.controls.targetYawMoment_Nm;
map.meta.wheelActuatorGroups = vehicle.powertrain.wheelActuatorGroups;
map.meta.frontRollMomentFraction = ...
    vehicle.suspension.frontRollMomentFraction;
map.meta.chassisTorsionalStiffness_NmPerRad = ...
    vehicle.chassis.torsionalStiffness_NmPerRad;
map.meta.boundaryIterations = options.boundaryIterations;
map.meta.interpolationSafetyFactor = options.interpolationSafetyFactor;
map.meta.lookupMethod = 'bilinear';
map.speedGrid_mps = speedGrid_mps;
map.lateralFractions = lateralFractions;
map.turnDirections = turnDirections;
map.maximumLateralAcceleration_mps2 = options.interpolationSafetyFactor ...
    * maximumLateralAcceleration_mps2;
map.maximumAcceleration_mps2 = options.interpolationSafetyFactor ...
    * maximumAcceleration_mps2;
map.maximumDeceleration_mps2 = options.interpolationSafetyFactor ...
    * maximumDeceleration_mps2;
map.boundaryStates.lateral = lateralBoundaryStates;
map.boundaryStates.acceleration = accelerationBoundaryStates;
map.boundaryStates.deceleration = decelerationBoundaryStates;
map.meta.boundaryStatesStored = true;
end

function [feasible, point] = query(vehicle, speed_mps, ax_mps2, ay_mps2, ...
    enableRetries, initialUnknowns)
[feasible, point] = lts.model.dualTrackSlip.isFeasible(vehicle, speed_mps, ...
    ax_mps2, ay_mps2, enableRetries=enableRetries, ...
    initialUnknowns=initialUnknowns);
end

function [lower, boundaryPoint] = findBoundary(feasibilityFunction, upper, iterationCount)
lower = 0;
boundaryPoint = struct.empty;
[feasible, point] = feasibilityFunction(lower, []);
if ~feasible
    return
end
boundaryPoint = point;
initialUnknowns = point.solver.solution;
for iteration = 1:iterationCount
    candidate = (lower + upper) / 2;
    [feasible, point] = feasibilityFunction(candidate, initialUnknowns);
    if feasible
        lower = candidate;
        boundaryPoint = point;
        initialUnknowns = point.solver.solution;
    else
        upper = candidate;
    end
end
end
