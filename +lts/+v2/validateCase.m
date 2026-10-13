function [config, report] = validateCase(config, options)
%VALIDATECASE Validate and resolve every effective V2 setting.

arguments
    config (1, 1) struct
    options.Display (1, 1) logical = true
end

assert(isfield(config, 'schema') && strcmp(config.schema, ...
    'lts.transparent.v2'), 'lts:v2:Schema', 'Unsupported V2 schema.');
lts.v2.modelRegistry(config.model);
lts.validation.validateVehicle(config.vehicle);
lts.validation.validateTrack(config.track);

featureNames = {'aero', 'loadTransfer', 'tireSlip', 'energy', ...
    'yawInertia', 'chassisTorsion'};
for index = 1:numel(featureNames)
    value = config.features.(featureNames{index});
    assert(islogical(value) && isscalar(value), 'lts:v2:FeatureType', ...
        'Feature %s must be an explicit scalar logical.', featureNames{index});
end

model = string(config.model);
strategy = string(config.solver.strategy);
assert(any(strategy == ["direct", "trackLocal", "mapped"]), ...
    'lts:v2:Strategy', 'Unknown solver strategy: %s.', strategy);
assert(config.solver.maximumSpeed_mps > 0, 'lts:v2:Speed', ...
    'maximumSpeed_mps must be positive.');
assert(config.solver.initialSpeed_mps >= 0 ...
    && config.solver.finalSpeed_mps >= 0, 'lts:v2:Speed', ...
    'Initial and final speed must be nonnegative.');

if model == "pointMass"
    assert(~config.features.tireSlip, 'lts:v2:UnsupportedFeature', ...
        'pointMass does not support tireSlip.');
    assert(~config.features.yawInertia && ~config.features.chassisTorsion, ...
        'lts:v2:UnsupportedFeature', ...
        'pointMass supports neither yawInertia nor chassisTorsion.');
elseif model == "singleTrack"
    assert(config.features.loadTransfer, 'lts:v2:UnsupportedFeature', ...
        'singleTrack requires loadTransfer=true; no hidden fallback exists.');
    assert(~config.features.chassisTorsion, 'lts:v2:UnsupportedFeature', ...
        'singleTrack cannot represent chassisTorsion.');
else
    assert(config.features.loadTransfer, 'lts:v2:UnsupportedFeature', ...
        'dualTrack requires loadTransfer=true.');
    assert(~config.features.chassisTorsion || config.features.tireSlip, ...
        'lts:v2:UnsupportedFeature', ...
        'chassisTorsion requires the dualTrack tireSlip formulation.');
end
if config.features.tireSlip
    assert(any(strategy == ["trackLocal", "mapped"]), ...
        'lts:v2:Strategy', 'tireSlip requires trackLocal or mapped.');
else
    assert(strategy == "direct", 'lts:v2:Strategy', ...
        'Cases without tireSlip require strategy=direct.');
end
assert(~config.features.yawInertia || ...
    (config.features.tireSlip && strategy == "trackLocal"), ...
    'lts:v2:UnsupportedFeature', ...
    'yawInertia requires tireSlip=true and strategy=trackLocal.');
if strategy == "mapped"
    assert(~isempty(config.solver.performanceMap), 'lts:v2:PerformanceMap', ...
        'strategy=mapped requires performanceMap.');
else
    assert(isempty(config.solver.performanceMap), 'lts:v2:PerformanceMap', ...
        'performanceMap must be empty unless strategy=mapped.');
end

effectiveVehicle = config.vehicle;
if ~config.features.aero
    names = {'dragCoefficient', 'totalLiftCoefficient', ...
        'liftCoefficientFront', 'liftCoefficientRear'};
    for index = 1:numel(names)
        effectiveVehicle.aerodynamics.(names{index}) = 0;
    end
end
if ~config.features.chassisTorsion
    effectiveVehicle.chassis.torsionalStiffness_NmPerRad = Inf;
else
    assert(isfinite(effectiveVehicle.chassis.torsionalStiffness_NmPerRad), ...
        'lts:v2:ChassisTorsion', ...
        'chassisTorsion=true requires a finite torsional stiffness.');
end
config.effectiveVehicle = effectiveVehicle;
[legacyModelId, legacyStrategy] = resolveModel(model, config.features, ...
    effectiveVehicle, strategy);
config.legacyModelId = char(legacyModelId);
config.legacyStrategy = char(legacyStrategy);
report = lts.v2.configurationReport(config);
if options.Display
    lts.v2.printConfiguration(config, report);
end
end

function [id, strategy] = resolveModel(model, features, vehicle, strategy)
if model == "pointMass"
    id = "pointMassAero";
    if features.loadTransfer
        strategy = "legacyQss";
    end
elseif model == "singleTrack"
    if features.tireSlip
        id = "singleTrackSlip";
    else
        id = "singleTrackAero";
    end
elseif features.tireSlip
    if string(vehicle.tireModel.active) == "fittedCombinedSlip"
        id = "dualTrackSlipFitted";
    elseif string(vehicle.tireModel.active) == "tmeasyAdtm"
        id = "dualTrackSlipTMeasy";
    else
        error('lts:v2:TireModel', ...
            'Unsupported selected tire model: %s.', vehicle.tireModel.active);
    end
else
    id = "dualTrackEnvelope";
end
end
