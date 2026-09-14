function model = create(vehicle)
%CREATE Build the selected tire-force model from vehicle configuration.

arguments
    vehicle (1, 1) struct
end

switch vehicle.tireModel.active
    case 'fittedCombinedSlip'
        configuration = vehicle.tireModel.fittedCombinedSlip;
        fit = lts.tire.loadFittedMagicFormula(string(configuration.file), ...
            configuration.lateralScale, configuration.longitudinalScale);
        model.type = 'fittedCombinedSlip';
        model.name = configuration.name;
        model.parameterSet = configuration.name;
        model.version = configuration.version;
        model.parameterVersion = configuration.version;
        model.parameters = fit;
    case 'tmeasyAdtm'
        configuration = vehicle.tireModel.tmeasyAdtm;
        parameters = lts.tire.tmeasyAdtm.loadParameters( ...
            configuration.parameterSet);
        model.type = 'tmeasyAdtm';
        model.name = configuration.name;
        model.parameterSet = configuration.parameterSet;
        model.version = configuration.version;
        model.parameterVersion = configuration.parameterVersion;
        model.parameters = parameters;
        model.muScale = configuration.muScale;
    otherwise
        error('lts:tire:UnknownModel', 'Unknown tire model: %s.', ...
            vehicle.tireModel.active);
end
end
