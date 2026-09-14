function comparison = runTireModelComparison(options)
%RUNTIREMODELCOMPARISON Compare fitted-curve and ADTM TMeasy boundaries.

arguments
    options.speedGrid_mps (1, :) double = [5, 10, 15, 20, 25, 30]
    options.lateralFractions (1, :) double = [0, 0.25, 0.5, 0.75, 1]
    options.boundaryIterations (1, 1) double = 10
    options.interpolationSafetyFactor (1, 1) double = 0.80
    options.showProgress (1, 1) logical = true
end

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot, fullfile(projectRoot, 'vehicles'));
trackFiles = lts.io.trackCatalog();
trackFile = trackFiles.endurance;
track = lts.io.loadTrackMatrix(trackFile, true);
modelNames = ["fittedCombinedSlip", "tmeasyAdtm"];
comparison = struct('tireModel', {}, 'parameterSet', {}, ...
    'lapTime_s', {}, 'netBatteryEnergy_kWh', {}, ...
    'performanceMap', {}, 'result', {});

for index = 1:numel(modelNames)
    vehicle = lts.tire.selectModel(FAUmaxTauV2("fourMotorAWD"), ...
        modelNames(index));
    performanceMap = lts.model.dualTrackSlip.buildPerformanceMap(vehicle, ...
        speedGrid_mps=options.speedGrid_mps, ...
        lateralFractions=options.lateralFractions, ...
        boundaryIterations=options.boundaryIterations, ...
        interpolationSafetyFactor=options.interpolationSafetyFactor, ...
        showProgress=options.showProgress);
    modelId = "dualTrackSlipFitted";
    if modelNames(index) == "tmeasyAdtm"
        modelId = "dualTrackSlipTMeasy";
    end
    config = lts.config.createCase(Vehicle=vehicle, Model=modelId, ...
        Strategy="mapped", PerformanceMap=performanceMap, ...
        IncludeEnergy=true, IncludeYawInertia=false, MaximumSpeed_mps=100, ...
        InitialSpeed_mps=0, FinalSpeed_mps=Inf);
    result = lts.design.runCase(config, track);
    energy = result.energy;
    comparison(index).tireModel = char(modelNames(index));
    comparison(index).parameterSet = performanceMap.meta.tireParameterSet;
    comparison(index).lapTime_s = result.lap.lapTime_s;
    comparison(index).netBatteryEnergy_kWh = energy.netBatteryEnergy_kWh;
    comparison(index).performanceMap = performanceMap;
    comparison(index).result = result;
    fprintf('%-24s %9.3f s  %8.4f kWh\n', modelNames(index), ...
        result.lap.lapTime_s, energy.netBatteryEnergy_kWh);
end

resultDirectory = fullfile(projectRoot, 'results');
if ~isfolder(resultDirectory)
    mkdir(resultDirectory);
end
resultFile = fullfile(resultDirectory, 'tireModelComparison.mat');
assert(~isfile(resultFile), 'lts:io:ResultExists', ...
    'Result file already exists; refusing to overwrite: %s', resultFile);
save(resultFile, 'comparison');
end
