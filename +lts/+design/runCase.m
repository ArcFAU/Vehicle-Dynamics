function result = runCase(config, track)
%RUNCASE Single V2 entry point for model selection and design outputs.

arguments
    config (1, 1) struct
    track (1, 1) struct
end

if isfield(config, 'showSummary')
    showSummary = config.showSummary;
else
    showSummary = false;
end
lts.config.validateCase(config, track, Display=showSummary);
vehicle = config.vehicle;
level = string(config.level);
strategy = string(config.strategy);
definition = lts.model.registry(config.modelId);
performanceMap = config.performanceMap;
trackLocalInfo = [];
if strategy == "trackLocal"
    [performanceMap, trackLocalInfo] = lts.design.buildTrackLocalMap(...
        vehicle, track, Model=string(config.modelId), ...
        MaximumSpeed_mps=config.maximumSpeed_mps, ...
        InitialSpeed_mps=config.initialSpeed_mps, ...
        FinalSpeed_mps=config.finalSpeed_mps, ...
        SpeedRowCount=config.trackLocal.speedRows, ...
        LateralFractions=config.trackLocal.lateralFractions, ...
        BoundaryIterations=config.trackLocal.boundaryIterations, ...
        InterpolationSafetyFactor=...
            config.trackLocal.interpolationSafetyFactor, ...
        ShowProgress=config.trackLocal.showProgress);
end
model = definition.factory(vehicle, strategy, performanceMap);
lap = lts.solver.solveLap(track, model, ...
    initialSpeed_mps=config.initialSpeed_mps, ...
    finalSpeed_mps=config.finalSpeed_mps, ...
    maximumSpeed_mps=config.maximumSpeed_mps);
if ~isempty(trackLocalInfo)
    trackLocalInfo.audit = auditTrackLocalLap(vehicle, definition, ...
        lap, model, config.trackLocal.auditPointCount, ...
        config.includeYawInertia);
end
result.meta.level = char(level);
result.meta.modelId = config.modelId;
result.meta.strategy = char(strategy);
result.meta.modelName = model.name;
result.meta.status = 'calculated';
resultConfiguration = config;
if ~isempty(config.performanceMap)
    resultConfiguration.performanceMap = struct( ...
        'meta', config.performanceMap.meta);
end
result.meta.configuration = resultConfiguration;
result.meta.physics.tireModel = vehicle.tireModel.active;
result.meta.physics.vehicleDataStatus = vehicle.meta.status;
result.meta.physics.execution = char(strategy);
result.meta.physics.energy = ternary(config.includeEnergy, ...
    'postProcessed', 'notRequested');
result.meta.physics.yawInertia = ternary(config.includeYawInertia, ...
    'criticalPointAudit', 'notIncluded');
result.meta.physics.usesPerformanceMap = any(strategy == ["mapped", "trackLocal"]);
if ~isempty(trackLocalInfo)
    result.meta.trackLocal = trackLocalInfo;
end
result.lap = lap;
result.brakeDemand = lts.analysis.brakeDemand(lap, vehicle);
result.limitClassification = lts.analysis.limitClassification(lap, model);
couplingMode = "diagnosticOnly";
if ~isempty(trackLocalInfo) && config.includeYawInertia
    couplingMode = "criticalPointAudit";
end
result.yawDemand = lts.analysis.yawDemand(lap, vehicle, ...
    CouplingMode=couplingMode);
if config.includeEnergy
    result.energy = lts.energy.analyzeLap(lap, vehicle);
end
result.capabilities = lts.design.capabilityMatrix();

function value = ternary(condition, trueValue, falseValue)
if condition
    value = trueValue;
else
    value = falseValue;
end
end
end

function audit = auditTrackLocalLap(vehicle, definition, lap, model, pointCount, ...
    includeYawInertia)
segmentCount = numel(lap.segmentAcceleration_mps2);
querySpeed_mps = zeros(segmentCount, 1);
queryLateralAcceleration_mps2 = zeros(segmentCount, 1);
for segment = 1:segmentCount
    if lap.segmentAcceleration_mps2(segment) >= 0
        point = segment;
    else
        point = segment + 1;
    end
    querySpeed_mps(segment) = lap.speed_mps(point);
    queryLateralAcceleration_mps2(segment) = ...
        lap.lateralAcceleration_mps2(point);
end
positiveSpeed = querySpeed_mps > 0;
candidateIndices = find(positiveSpeed);
yaw = lts.analysis.yawDemand(lap, vehicle);
targetYawMoment_Nm = zeros(size(querySpeed_mps));
if includeYawInertia
    for segment = 1:segmentCount
        if lap.segmentAcceleration_mps2(segment) >= 0
            point = segment;
        else
            point = segment + 1;
        end
        targetYawMoment_Nm(segment) = yaw.inertialMoment_Nm(point);
    end
end
if isempty(candidateIndices) || pointCount == 0
    audit.checkedPointCount = 0;
    audit.allConfirmedFeasible = true;
    audit.includesYawInertia = includeYawInertia;
    return
end

limits = model.accelerationLimits(querySpeed_mps(positiveSpeed), ...
    queryLateralAcceleration_mps2(positiveSpeed));
candidateAcceleration_mps2 = lap.segmentAcceleration_mps2(positiveSpeed);
utilization = zeros(size(candidateAcceleration_mps2));
accelerating = candidateAcceleration_mps2 >= 0;
utilization(accelerating) = candidateAcceleration_mps2(accelerating) ...
    ./ max(limits.maximumAcceleration_mps2(accelerating), eps);
utilization(~accelerating) = -candidateAcceleration_mps2(~accelerating) ...
    ./ max(limits.maximumDeceleration_mps2(~accelerating), eps);
[~, order] = sort(utilization, 'descend');
selected = candidateIndices(order(1:min(pointCount, numel(order))));
[~, maximumLateralIndex] = max(abs(queryLateralAcceleration_mps2));
[~, maximumSpeedIndex] = max(querySpeed_mps);
[~, maximumYawMomentIndex] = max(abs(targetYawMoment_Nm));
selected = unique([selected; maximumLateralIndex; maximumSpeedIndex; ...
    maximumYawMomentIndex]);
selected = selected(querySpeed_mps(selected) > 0);

solverVehicle = vehicle;
if definition.mapType == "singleTrack"
    solverVehicle = lts.model.singleTrackSlip.equivalentVehicle(vehicle);
end
confirmed = false(size(selected));
initialUnknowns = [];
for index = 1:numel(selected)
    segment = selected(index);
    [confirmed(index), point] = lts.model.dualTrackSlip.isFeasible(...
        solverVehicle, querySpeed_mps(segment), ...
        lap.segmentAcceleration_mps2(segment), ...
        queryLateralAcceleration_mps2(segment), enableRetries=true, ...
        targetYawMoment_Nm=targetYawMoment_Nm(segment), ...
        initialUnknowns=initialUnknowns);
    if confirmed(index)
        initialUnknowns = point.solver.solution;
    else
        initialUnknowns = [];
    end
end

audit.checkedPointCount = numel(selected);
audit.allConfirmedFeasible = all(confirmed);
audit.segmentIndices = selected;
audit.includesYawInertia = includeYawInertia;
audit.targetYawMoments_Nm = targetYawMoment_Nm(selected);
audit.maximumAbsoluteTargetYawMoment_Nm = ...
    max(abs(audit.targetYawMoments_Nm), [], 'all');
assert(audit.allConfirmedFeasible, 'lts:design:TrackLocalAudit', ...
    ['The nonlinear solver could not confirm every critical track-local ' ...
    'operating point. The result is not accepted as a valid lap.']);
end
