function configurations = availableConfigurations(options)
%AVAILABLECONFIGURATIONS List registered model/strategy combinations.

arguments
    options.Model (1, 1) string = ""
    options.Strategy (1, 1) string = ""
end

definitions = lts.model.registry();
modelId = strings(0, 1);
level = strings(0, 1);
strategy = strings(0, 1);
mapRequired = false(0, 1);
mapType = strings(0, 1);
tireRequirement = strings(0, 1);
supportsYawAudit = false(0, 1);
operatingPointType = strings(0, 1);
for definitionIndex = 1:numel(definitions)
    definition = definitions(definitionIndex);
    for strategyIndex = 1:numel(definition.strategies)
        selectedStrategy = definition.strategies(strategyIndex);
        modelId(end + 1, 1) = definition.id; %#ok<AGROW>
        level(end + 1, 1) = definition.level; %#ok<AGROW>
        strategy(end + 1, 1) = selectedStrategy; %#ok<AGROW>
        mapRequired(end + 1, 1) = selectedStrategy == "mapped"; %#ok<AGROW>
        mapType(end + 1, 1) = definition.mapType; %#ok<AGROW>
        tireRequirement(end + 1, 1) = definition.tireRequirement; %#ok<AGROW>
        supportsYawAudit(end + 1, 1) = ...
            definition.supportsYawAudit; %#ok<AGROW>
        operatingPointType(end + 1, 1) = ...
            definition.operatingPointType; %#ok<AGROW>
    end
end
configurations = table(modelId, level, strategy, mapRequired, mapType, ...
    tireRequirement, supportsYawAudit, operatingPointType);
if options.Model ~= ""
    configurations = configurations(configurations.modelId == options.Model, :);
end
if options.Strategy ~= ""
    configurations = configurations(...
        configurations.strategy == options.Strategy, :);
end
assert(~isempty(configurations), 'lts:model:AvailableConfigurations', ...
    'No registered configuration matches the requested filters.');
if nargout == 0
    disp(configurations);
    clear configurations
end
end
