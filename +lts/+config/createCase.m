function config = createCase(options)
%CREATECASE Build one explicit, inspectable simulation configuration.

arguments
    options.Vehicle (1, 1) struct
    options.Model (1, 1) string
    options.Strategy (1, 1) string
    options.IncludeEnergy (1, 1) logical
    options.IncludeYawInertia (1, 1) logical
    options.MaximumSpeed_mps (1, 1) double {mustBePositive}
    options.InitialSpeed_mps (1, 1) double {mustBeNonnegative}
    options.FinalSpeed_mps (1, 1) double {mustBeNonnegative}
    options.PerformanceMap = []
    options.TrackLocalSpeedRows (1, 1) double ...
        {mustBeInteger, mustBeGreaterThanOrEqual(options.TrackLocalSpeedRows, 2)} = 3
    options.TrackLocalLateralFractions (1, :) double = []
    options.TrackLocalBoundaryIterations (1, 1) double ...
        {mustBeInteger, mustBePositive} = 6
    options.TrackLocalInterpolationSafetyFactor (1, 1) double = NaN
    options.TrackLocalShowProgress (1, 1) logical = false
    options.TrackLocalAuditPointCount (1, 1) double ...
        {mustBeInteger, mustBeNonnegative} = 8
    options.ShowSummary (1, 1) logical = true
end

requiredSelections = {'Vehicle', 'Model', 'Strategy', 'IncludeEnergy', ...
    'IncludeYawInertia', 'MaximumSpeed_mps', 'InitialSpeed_mps', ...
    'FinalSpeed_mps'};
for index = 1:numel(requiredSelections)
    assert(isfield(options, requiredSelections{index}), ...
        'lts:config:MissingSelection', ...
        'Explicit selection is required: %s.', requiredSelections{index});
end
lts.validation.validateVehicle(options.Vehicle);
definition = lts.model.registry(options.Model);
assert(any(options.Strategy == definition.strategies), ...
    'lts:config:ModelStrategy', ...
    'Strategy "%s" is unavailable for model "%s".', ...
    options.Strategy, options.Model);
assert(~options.IncludeYawInertia || ...
    (options.Strategy == "trackLocal" ...
        && definition.supportsYawAudit), ...
    'lts:config:YawInertia', ...
    ['IncludeYawInertia is only implemented for the critical-point audit ' ...
    'of track-local slip models.']);
if any(definition.tireRequirement == ...
        ["fittedCombinedSlip", "tmeasyAdtm"])
    assert(string(options.Vehicle.tireModel.active) == ...
        definition.tireRequirement, 'lts:config:TireModel', ...
        'Model "%s" requires tire model "%s", but vehicle selects "%s".', ...
        options.Model, definition.tireRequirement, ...
        options.Vehicle.tireModel.active);
end
if options.Strategy == "mapped"
    assert(~isempty(options.PerformanceMap), 'lts:config:PerformanceMap', ...
        'Strategy "mapped" requires an explicit PerformanceMap.');
else
    assert(isempty(options.PerformanceMap), 'lts:config:PerformanceMap', ...
        'PerformanceMap is only valid with Strategy="mapped".');
end

config.schema = 'lts.case.v1';
config.vehicle = options.Vehicle;
config.modelId = char(definition.id);
config.level = char(definition.level);
config.strategy = char(options.Strategy);
config.includeEnergy = options.IncludeEnergy;
config.includeYawInertia = options.IncludeYawInertia;
config.maximumSpeed_mps = options.MaximumSpeed_mps;
config.initialSpeed_mps = options.InitialSpeed_mps;
config.finalSpeed_mps = options.FinalSpeed_mps;
config.performanceMap = options.PerformanceMap;
config.trackLocal.speedRows = options.TrackLocalSpeedRows;
config.trackLocal.lateralFractions = options.TrackLocalLateralFractions;
config.trackLocal.boundaryIterations = options.TrackLocalBoundaryIterations;
config.trackLocal.interpolationSafetyFactor = ...
    options.TrackLocalInterpolationSafetyFactor;
config.trackLocal.showProgress = options.TrackLocalShowProgress;
config.trackLocal.auditPointCount = options.TrackLocalAuditPointCount;
config.showSummary = options.ShowSummary;
end
