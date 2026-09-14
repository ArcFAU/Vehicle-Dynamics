function trace = operatingPointTrace(result, vehicle, options)
%OPERATINGPOINTTRACE Recalculate selected lap states for engineering output.

arguments
    result (1, 1) struct
    vehicle (1, 1) struct
    options.PointCount (1, 1) double ...
        {mustBeInteger, mustBePositive} = 20
    options.IncludeYawInertia (1, 1) logical = false
end

lap = result.lap;
segmentCount = numel(lap.segmentAcceleration_mps2);
segments = unique(round(linspace(1, segmentCount, ...
    min(options.PointCount, segmentCount))));
yaw = lts.analysis.yawDemand(lap, vehicle);
points = cell(numel(segments), 1);
confirmed = false(numel(segments), 1);
initialUnknowns = [];
modelId = resultModelId(result);
definition = lts.model.registry(modelId);
for index = 1:numel(segments)
    segment = segments(index);
    pointIndex = segment;
    if lap.segmentAcceleration_mps2(segment) < 0
        pointIndex = segment + 1;
    end
    targetYawMoment_Nm = 0;
    if options.IncludeYawInertia
        targetYawMoment_Nm = yaw.inertialMoment_Nm(pointIndex);
    end
    if definition.operatingPointType == "singleTrackSlip"
        point = lts.model.singleTrackSlip.operatingPoint(vehicle, ...
            lap.speed_mps(pointIndex), ...
            lap.segmentAcceleration_mps2(segment), ...
            lap.lateralAcceleration_mps2(pointIndex), ...
            targetYawMoment_Nm=targetYawMoment_Nm, ...
            initialUnknowns=initialUnknowns);
    elseif definition.operatingPointType == "dualTrackSlip"
        point = lts.model.dualTrackSlip.operatingPoint(vehicle, ...
            lap.speed_mps(pointIndex), ...
            lap.segmentAcceleration_mps2(segment), ...
            lap.lateralAcceleration_mps2(pointIndex), ...
            targetYawMoment_Nm=targetYawMoment_Nm, ...
            initialUnknowns=initialUnknowns);
    elseif definition.operatingPointType == "singleTrackAero"
        point = lts.model.singleTrackAero.operatingPoint(vehicle, ...
            lap.speed_mps(pointIndex), ...
            lap.segmentAcceleration_mps2(segment), ...
            lap.lateralAcceleration_mps2(pointIndex));
    else
        error('lts:analysis:OperatingPointModel', ...
            'Operating-point traces are unavailable for model "%s".', modelId);
    end
    points{index} = point;
    confirmed(index) = point.feasible;
    if isfield(point, 'solver') && point.feasible
        initialUnknowns = point.solver.solution;
    else
        initialUnknowns = [];
    end
end
trace.segmentIndices = segments(:);
trace.distance_m = lap.distance_m(segments(:));
trace.points = points;
trace.confirmedFeasible = confirmed;
trace.allConfirmedFeasible = all(confirmed);
trace.includesYawInertia = options.IncludeYawInertia;
end

function modelId = resultModelId(result)
if isfield(result.meta, 'modelId')
    modelId = string(result.meta.modelId);
    return
end
assert(isfield(result.meta, 'level'), ...
    'lts:analysis:OperatingPointModel', ...
    'Result metadata contains neither modelId nor legacy level.');
legacyLevel = string(result.meta.level);
legacyModelIds = dictionary(["1.2", "1.3", "2.1", "2.5"], ...
    ["singleTrackAero", "singleTrackSlip", ...
    "dualTrackSlipFitted", "dualTrackSlipTMeasy"]);
assert(isKey(legacyModelIds, legacyLevel), ...
    'lts:analysis:OperatingPointModel', ...
    'No model ID mapping exists for legacy level "%s".', legacyLevel);
modelId = legacyModelIds(legacyLevel);
end
