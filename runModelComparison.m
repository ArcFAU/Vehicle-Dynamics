function comparison = runModelComparison(modelIds, slipPerformanceMap)
%RUNMODELCOMPARISON Compare implemented model levels on one track geometry.

arguments
    modelIds (1, :) string
    slipPerformanceMap = []
end

assert(~isempty(modelIds), 'lts:runner:ModelIds', ...
    'Specify one or more registered model IDs.');
assert(numel(unique(modelIds)) == numel(modelIds), ...
    'lts:runner:ModelIds', 'Model IDs must not contain duplicates.');
definitions = arrayfun(@lts.model.registry, modelIds);
needsMap = [definitions.mapType] ~= "none";
if any(needsMap)
    assert(~isempty(slipPerformanceMap), 'lts:runner:MissingPerformanceMap', ...
        'A requested slip model requires an explicit performance map.');
else
    assert(isempty(slipPerformanceMap), 'lts:runner:UnusedPerformanceMap', ...
        'A performance map was supplied although no slip model was requested.');
end

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot, fullfile(projectRoot, 'vehicles'));
vehicle = FAUmaxTauV2("fourMotorAWD");
trackFiles = lts.io.trackCatalog();
trackFile = trackFiles.endurance;
track = lts.io.loadTrackMatrix(trackFile, true);

comparison = struct('name', {}, 'level', {}, 'lapTime_s', {}, 'result', {});
for index = 1:numel(definitions)
    strategy = "direct";
    performanceMap = [];
    if definitions(index).mapType ~= "none"
        strategy = "mapped";
        performanceMap = slipPerformanceMap;
    end
    config = lts.config.createCase(Vehicle=vehicle, ...
        Model=definitions(index).id, Strategy=strategy, ...
        PerformanceMap=performanceMap, IncludeEnergy=false, ...
        IncludeYawInertia=false, MaximumSpeed_mps=100, ...
        InitialSpeed_mps=0, FinalSpeed_mps=Inf);
    caseResult = lts.design.runCase(config, track);
    comparison(index).name = caseResult.meta.modelName;
    comparison(index).level = caseResult.meta.level;
    comparison(index).lapTime_s = caseResult.lap.lapTime_s;
    comparison(index).result = caseResult;
    fprintf('%-38s %9.3f s\n', comparison(index).name, ...
        comparison(index).lapTime_s);
end

figure('Name', 'nextGen model comparison');
hold on;
for index = 1:numel(comparison)
    plot(comparison(index).result.lap.distance_m, ...
        comparison(index).result.lap.speed_mps * 3.6, 'LineWidth', 1.4, ...
        'DisplayName', comparison(index).name);
end
grid on;
xlabel('Distance [m]');
ylabel('Speed [km/h]');
title('Implemented Model Levels');
legend('Location', 'best');
end
