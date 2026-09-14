function study = runSensitivityStudy(caseConfig)
%RUNSENSITIVITYSTUDY Example mass sensitivity across selected FSG events.
% Use config/sensitivityStudyCase.m or pass a copied, adjusted case struct.

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot, fullfile(projectRoot, 'vehicles'), ...
    fullfile(projectRoot, 'config'));
assert(nargin == 1 && isstruct(caseConfig), ...
    'lts:study:ExplicitConfigurationRequired', ...
    ['Pass an explicit configuration from sensitivityStudyCase or an ' ...
    'equivalent fully specified struct.']);
requiredFields = {'name', 'model', 'includeDriverless', 'coupleEnergy', ...
    'baseVehicle', 'createPlots', 'parameters', 'events', 'scoring'};
for fieldIndex = 1:numel(requiredFields)
    assert(isfield(caseConfig, requiredFields{fieldIndex}), ...
        'lts:study:ExplicitConfigurationRequired', ...
        'Missing explicit study configuration field: %s.', ...
        requiredFields{fieldIndex});
end

modelFactory = selectModelFactory(caseConfig.model);
study = lts.study.runParameterSweep(caseConfig.baseVehicle, ...
    caseConfig.parameters, caseConfig.events, modelFactory, ...
    caseConfig.scoring, ...
    IncludeDriverless=caseConfig.includeDriverless, ...
    CoupleEnergy=caseConfig.coupleEnergy);

fprintf('Sensitivity study: %d cases, driverless=%d\n', ...
    numel(study.cases), caseConfig.includeDriverless);
for index = 1:numel(study.cases)
    fprintf('Case %3d  points=%8.3f\n', index, study.totalPoints(index));
end
if caseConfig.createPlots
    study.figures.sensitivity = plotSensitivity(study, caseConfig.name);
else
    study.figures.sensitivity = gobjects(0);
end
end

function modelFactory = selectModelFactory(modelName)
definition = lts.model.registry(string(modelName));
assert(any(definition.strategies == "direct") ...
    && definition.mapType == "none", 'lts:study:Model', ...
    'Sensitivity model must be a registered direct model without a map.');
modelFactory = @(vehicle) definition.factory(vehicle, "direct", []);
end

function figureHandle = plotSensitivity(study, studyName)
parameterCount = numel(study.parameters);
if parameterCount == 1
    figureHandle = plotOneDimensional(study, studyName);
elseif parameterCount == 2
    figureHandle = plotTwoDimensional(study, studyName);
else
    warning('lts:study:PlotDimensions', ...
        'Automatic plots support one or two parameter axes.');
    figureHandle = gobjects(0);
end
end

function figureHandle = plotOneDimensional(study, studyName)
x = study.parameterValues(:, 1);
parameter = study.parameters(1);
figureHandle = figure('Name', ['Sensitivity: ', studyName]);
layout = tiledlayout(2, 2, 'TileSpacing', 'compact', ...
    'Padding', 'compact');
title(layout, strrep(studyName, '_', '\_'));

nexttile;
plot(x, study.relative.eventTime_percent, 'LineWidth', 1.4);
yline(0, ':k');
grid on;
xlabel(axisLabel(parameter));
ylabel('Time change from baseline [%]');
legend(study.eventIds, 'Interpreter', 'none', 'Location', 'best');

[eventPoints, scoreNames] = eventPointMatrix(study.cases);
nexttile;
area(x, eventPoints);
grid on;
xlabel(axisLabel(parameter));
ylabel('Event points');
legend(scoreNames, 'Interpreter', 'none', 'Location', 'best');

nexttile;
plot(x, study.totalPoints, 'k', 'LineWidth', 1.8);
hold on;
baselineIndex = study.sensitivity.baselineCaseIndex;
baselineX = x(baselineIndex);
baselinePoints = study.totalPoints(baselineIndex);
slope = study.sensitivity.pointsPerUnit(1);
if isfinite(baselinePoints) && isfinite(slope)
    plot(x, baselinePoints + slope * (x - baselineX), '--r', ...
        'LineWidth', 1.2);
    xline(baselineX, ':r');
    legend('Total points', sprintf('Local slope: %.4g points/%s', ...
        slope, parameter.unit), 'Location', 'best');
end
grid on;
xlabel(axisLabel(parameter));
ylabel('Total points');

nexttile;
plot(x, study.netBatteryEnergy_kWh, 'LineWidth', 1.4);
grid on;
xlabel(axisLabel(parameter));
ylabel('Net battery energy [kWh]');
legend(study.eventIds, 'Interpreter', 'none', 'Location', 'best');
end

function figureHandle = plotTwoDimensional(study, studyName)
first = study.parameters(1);
second = study.parameters(2);
x = first.values;
y = second.values;
if any(isfinite(study.totalPoints))
    values = study.totalPoints;
    colorLabel = 'Total points';
else
    values = study.eventTimes_s(:, 1);
    colorLabel = [study.eventIds{1}, ' time [s]'];
end
z = reshape(values, numel(x), numel(y)).';
figureHandle = figure('Name', ['Sensitivity map: ', studyName]);
contourf(x, y, z, 16, 'ShowText', 'on');
colorbar;
grid on;
xlabel(axisLabel(first));
ylabel(axisLabel(second));
title({strrep(studyName, '_', '\_'), colorLabel});
hold on;
plot(first.baseline, second.baseline, 'xr', ...
    'MarkerSize', 10, 'LineWidth', 2);
end

function label = axisLabel(parameter)
label = parameter.name;
if ~isempty(parameter.unit)
    label = sprintf('%s [%s]', label, parameter.unit);
end
end

function [values, names] = eventPointMatrix(cases)
names = {cases(1).scoring.events.name};
values = NaN(numel(cases), numel(names));
for caseIndex = 1:numel(cases)
    scores = cases(caseIndex).scoring.events;
    values(caseIndex, :) = [scores.points];
end
end
