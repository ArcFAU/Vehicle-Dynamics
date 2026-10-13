function report = configurationReport(config)
%CONFIGURATIONREPORT Classify supplied inputs and expose effective settings.

paths = flatten(config.vehicle, "vehicle");
model = string(config.model);
usedPrefixes = ["vehicle.mass", "vehicle.tireEnvelope", ...
    "vehicle.tireModel", "vehicle.powertrain", "vehicle.brakes"];
if config.features.aero
    usedPrefixes(end + 1) = "vehicle.aerodynamics";
end
if model ~= "pointMass" || config.features.loadTransfer
    usedPrefixes(end + 1) = "vehicle.geometry";
end
if model == "dualTrack"
    usedPrefixes(end + 1) = "vehicle.controls";
    usedPrefixes(end + 1) = "vehicle.suspension";
end
if config.features.tireSlip
    usedPrefixes(end + 1) = "vehicle.tireModel";
    usedPrefixes(end + 1) = "vehicle.tireSlipModel";
    usedPrefixes(end + 1) = "vehicle.tires";
end
if config.features.energy
    usedPrefixes(end + 1) = "vehicle.energy";
end
if config.features.chassisTorsion
    usedPrefixes(end + 1) = "vehicle.chassis";
end

unsupportedPrefixes = strings(0, 1);
if model == "pointMass"
    unsupportedPrefixes = ["vehicle.suspension", "vehicle.chassis", ...
        "vehicle.controls", "vehicle.tireSlipModel", "vehicle.tires"];
elseif model == "singleTrack"
    unsupportedPrefixes = ["vehicle.suspension", "vehicle.chassis"];
end

status = repmat("ignored", size(paths));
for index = 1:numel(paths)
    if any(startsWith(paths(index), usedPrefixes + ".") ...
            | paths(index) == usedPrefixes)
        status(index) = "used";
    elseif any(startsWith(paths(index), unsupportedPrefixes + ".") ...
            | paths(index) == unsupportedPrefixes)
        status(index) = "notSupported";
    end
end
report.inputs = table(paths(:), status(:), ...
    'VariableNames', {'path', 'status'});
report.model = string(config.model);
report.solverStrategy = string(config.solver.strategy);
report.calculationStrategy = string(config.legacyStrategy);
report.calculationKernel = string(config.legacyModelId);
report.features = struct2table(config.features, 'AsArray', true);
report.effectiveVehicle = config.effectiveVehicle;
report.notes = [ ...
    "Aero coefficients are forced to zero when aero=false."; ...
    "Unsupported active features are rejected, never ignored."; ...
    "The selected calculation kernel and strategy are printed explicitly."; ...
    "Ignored and unsupported vehicle inputs remain visible in report.inputs."];
end

function paths = flatten(value, prefix)
paths = strings(0, 1);
fields = fieldnames(value);
for index = 1:numel(fields)
    path = prefix + "." + string(fields{index});
    child = value.(fields{index});
    if isstruct(child) && isscalar(child)
        paths = [paths; flatten(child, path)]; %#ok<AGROW>
    else
        paths(end + 1, 1) = path; %#ok<AGROW>
    end
end
end
