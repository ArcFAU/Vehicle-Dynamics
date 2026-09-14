function study = runParameterSweep(baseVehicle, parameters, events, ...
        modelFactory, ruleSet, options)
%RUNPARAMETERSWEEP Run a full-factorial laptime and points sensitivity study.

arguments
    baseVehicle (1, 1) struct
    parameters (1, :) struct
    events (1, :) struct
    modelFactory (1, 1) function_handle
    ruleSet (1, 1) struct
    options.IncludeDriverless (1, 1) logical
    options.CoupleEnergy (1, 1) logical
end

assert(~isempty(parameters), 'lts:study:Parameters', ...
    'At least one sweep parameter is required.');
valueCounts = arrayfun(@(item) numel(item.values), parameters);
caseCount = prod(valueCounts);
parameterValues = zeros(caseCount, numel(parameters));
cases = struct('vehicle', {}, 'eventResults', {}, 'scoring', {});

for linearIndex = 1:caseCount
    if isscalar(parameters)
        subscripts = {linearIndex};
    else
        subscripts = cell(1, numel(parameters));
        [subscripts{:}] = ind2sub(valueCounts, linearIndex);
    end
    vehicle = baseVehicle;
    for parameterIndex = 1:numel(parameters)
        value = parameters(parameterIndex).values(subscripts{parameterIndex});
        parameterValues(linearIndex, parameterIndex) = value;
        vehicle = lts.study.applyParameter(vehicle, ...
            parameters(parameterIndex), value);
    end
    model = modelFactory(vehicle);
    eventResults = lts.events.runSet(events, model, ...
        IncludeDriverless=options.IncludeDriverless, ...
        CoupleEnergy=options.CoupleEnergy);
    scoring = lts.scoring.scoreEvents(eventResults, ruleSet, ...
        IncludeDriverless=options.IncludeDriverless);
    cases(linearIndex).vehicle = vehicle;
    cases(linearIndex).eventResults = eventResults;
    cases(linearIndex).scoring = scoring;
end

study.meta.scoring = ruleSet.meta;
study.meta.includeDriverless = options.IncludeDriverless;
study.meta.energyCoupled = options.CoupleEnergy;
study.parameters = parameters;
study.parameterValues = parameterValues;
study.cases = cases;
study.totalPoints = arrayfun(@(item) item.scoring.totalPoints, cases).';
study.eventIds = {cases(1).eventResults.id};
study.eventTimes_s = extractEventMetric(cases, 'time_s');
study.netBatteryEnergy_kWh = extractEnergyMetric(cases, ...
    'netBatteryEnergy_kWh');
study.recoveredEnergy_kWh = extractEnergyMetric(cases, ...
    'recoveredEnergy_kWh');
study.requiredInstalledCapacity_kWh = extractEnergyMetric(cases, ...
    'requiredInstalledCapacity_kWh');
study.peakBatteryDischargePower_kW = extractEnergyMetric(cases, ...
    'peakBatteryDischargePower_kW');
study.peakBatteryChargePower_kW = extractEnergyMetric(cases, ...
    'peakBatteryChargePower_kW');
study.penalty = performancePenalty(study);
study.sensitivity = localSensitivity(study, valueCounts);
baselineIndex = study.sensitivity.baselineCaseIndex;
study.relative.baselineEventTimes_s = study.eventTimes_s(baselineIndex, :);
study.relative.eventTime_percent = 100 * (...
    study.eventTimes_s ./ study.relative.baselineEventTimes_s - 1);
study.relative.baselineTotalPoints = study.totalPoints(baselineIndex);
study.relative.totalPointsChange = study.totalPoints ...
    - study.relative.baselineTotalPoints;
study.relative.netBatteryEnergy_percent = 100 * (...
    study.netBatteryEnergy_kWh ...
    ./ study.netBatteryEnergy_kWh(baselineIndex, :) - 1);
end

function penalty = performancePenalty(study)
%PERFORMANCEPENALTY Express every concept relative to the best sweep case.

penalty.bestEventTime_s = min(study.eventTimes_s, [], 1, 'omitnan');
penalty.eventTime_s = study.eventTimes_s - penalty.bestEventTime_s;
[~, penalty.fastestCaseIndex] = min(study.eventTimes_s, [], 1, 'omitnan');

finitePoints = study.totalPoints(isfinite(study.totalPoints));
if isempty(finitePoints)
    penalty.bestTotalPoints = NaN;
    penalty.totalPointsLoss = NaN(size(study.totalPoints));
    penalty.bestPointsCaseIndex = NaN;
else
    penalty.bestTotalPoints = max(finitePoints);
    penalty.totalPointsLoss = penalty.bestTotalPoints - study.totalPoints;
    [~, penalty.bestPointsCaseIndex] = max(study.totalPoints, [], 'omitnan');
end
end

function values = extractEventMetric(cases, field)
eventCount = numel(cases(1).eventResults);
values = NaN(numel(cases), eventCount);
for caseIndex = 1:numel(cases)
    for eventIndex = 1:eventCount
        values(caseIndex, eventIndex) = ...
            cases(caseIndex).eventResults(eventIndex).(field);
    end
end
end

function values = extractEnergyMetric(cases, field)
eventCount = numel(cases(1).eventResults);
values = NaN(numel(cases), eventCount);
for caseIndex = 1:numel(cases)
    for eventIndex = 1:eventCount
        energy = cases(caseIndex).eventResults(eventIndex).energy;
        if isfield(energy, field)
            values(caseIndex, eventIndex) = energy.(field);
        end
    end
end
end

function sensitivity = localSensitivity(study, valueCounts)
parameters = study.parameters;
baselineSubscripts = cell(1, numel(parameters));
for index = 1:numel(parameters)
    values = parameters(index).values;
    baseline = parameters(index).baseline;
    if ~isfinite(baseline)
        baseline = values(ceil(numel(values) / 2));
    end
    [~, baselineSubscripts{index}] = min(abs(values - baseline));
end
baselineIndex = toLinearIndex(valueCounts, baselineSubscripts);
baselinePoints = study.totalPoints(baselineIndex);

sensitivity.baselineCaseIndex = baselineIndex;
sensitivity.baselinePoints = baselinePoints;
sensitivity.pointsPerUnit = NaN(1, numel(parameters));
sensitivity.eventTime_sPerUnit = ...
    NaN(numel(parameters), numel(study.eventIds));
sensitivity.netBatteryEnergy_kWhPerUnit = ...
    NaN(numel(parameters), numel(study.eventIds));
sensitivity.recoveredEnergy_kWhPerUnit = ...
    NaN(numel(parameters), numel(study.eventIds));
sensitivity.requiredInstalledCapacity_kWhPerUnit = ...
    NaN(numel(parameters), numel(study.eventIds));
for parameterIndex = 1:numel(parameters)
    lowerSubscripts = baselineSubscripts;
    upperSubscripts = baselineSubscripts;
    lowerSubscripts{parameterIndex} = max(1, ...
        baselineSubscripts{parameterIndex} - 1);
    upperSubscripts{parameterIndex} = min(valueCounts(parameterIndex), ...
        baselineSubscripts{parameterIndex} + 1);
    if lowerSubscripts{parameterIndex} == upperSubscripts{parameterIndex}
        continue
    end
    lowerIndex = toLinearIndex(valueCounts, lowerSubscripts);
    upperIndex = toLinearIndex(valueCounts, upperSubscripts);
    lowerValue = parameters(parameterIndex).values( ...
        lowerSubscripts{parameterIndex});
    upperValue = parameters(parameterIndex).values( ...
        upperSubscripts{parameterIndex});
    sensitivity.pointsPerUnit(parameterIndex) = ...
        (study.totalPoints(upperIndex) - study.totalPoints(lowerIndex)) ...
        / (upperValue - lowerValue);
    sensitivity.eventTime_sPerUnit(parameterIndex, :) = ...
        (study.eventTimes_s(upperIndex, :) ...
        - study.eventTimes_s(lowerIndex, :)) / (upperValue - lowerValue);
    sensitivity.netBatteryEnergy_kWhPerUnit(parameterIndex, :) = ...
        (study.netBatteryEnergy_kWh(upperIndex, :) ...
        - study.netBatteryEnergy_kWh(lowerIndex, :)) ...
        / (upperValue - lowerValue);
    sensitivity.recoveredEnergy_kWhPerUnit(parameterIndex, :) = ...
        (study.recoveredEnergy_kWh(upperIndex, :) ...
        - study.recoveredEnergy_kWh(lowerIndex, :)) ...
        / (upperValue - lowerValue);
    sensitivity.requiredInstalledCapacity_kWhPerUnit(parameterIndex, :) = ...
        (study.requiredInstalledCapacity_kWh(upperIndex, :) ...
        - study.requiredInstalledCapacity_kWh(lowerIndex, :)) ...
        / (upperValue - lowerValue);
end
end

function linearIndex = toLinearIndex(valueCounts, subscripts)
if isscalar(valueCounts)
    linearIndex = subscripts{1};
else
    linearIndex = sub2ind(valueCounts, subscripts{:});
end
end
