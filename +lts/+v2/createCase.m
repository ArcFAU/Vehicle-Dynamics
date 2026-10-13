function config = createCase(input)
%CREATECASE Create one explicit, transparent V2 simulation case.

arguments
    input (1, 1) struct
end

required = {'vehicle', 'track', 'model', 'features', 'solver'};
requireFields(input, required, 'input');
requireFields(input.features, {'aero', 'loadTransfer', 'tireSlip', ...
    'energy', 'yawInertia', 'chassisTorsion'}, 'input.features');
requireFields(input.solver, {'strategy', 'maximumSpeed_mps', ...
    'initialSpeed_mps', 'finalSpeed_mps', 'performanceMap', ...
    'trackLocal'}, 'input.solver');
requireFields(input.solver.trackLocal, {'speedRows', 'lateralFractions', ...
    'boundaryIterations', 'interpolationSafetyFactor', 'showProgress', ...
    'auditPointCount'}, 'input.solver.trackLocal');

config = input;
config.schema = 'lts.transparent.v2';
config.model = char(string(config.model));
config.solver.strategy = char(string(config.solver.strategy));
config = lts.v2.validateCase(config, Display=false);
end

function requireFields(value, fields, location)
for index = 1:numel(fields)
    assert(isfield(value, fields{index}), 'lts:v2:MissingInput', ...
        'Explicit input required: %s.%s.', location, fields{index});
end
end
