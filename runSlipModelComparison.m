function comparison = runSlipModelComparison()
%RUNSLIPMODELCOMPARISON Build Level 2.1c map and compare it with Level 2.0.

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot, fullfile(projectRoot, 'vehicles'));
vehicle = FAUmaxTauV2("fourMotorAWD");
trackFiles = lts.io.trackCatalog();
trackFile = trackFiles.endurance;
track = lts.io.loadTrackMatrix(trackFile, true);
performanceMap = lts.model.dualTrackSlip.buildPerformanceMap(vehicle);
configs = {lts.config.createCase(Vehicle=vehicle, ...
    Model="dualTrackEnvelope", Strategy="direct", IncludeEnergy=false, ...
    IncludeYawInertia=false, MaximumSpeed_mps=100, InitialSpeed_mps=0, ...
    FinalSpeed_mps=Inf), ...
    lts.config.createCase(Vehicle=vehicle, Model="dualTrackSlipFitted", ...
    Strategy="mapped", PerformanceMap=performanceMap, IncludeEnergy=false, ...
    IncludeYawInertia=false, MaximumSpeed_mps=100, InitialSpeed_mps=0, ...
    FinalSpeed_mps=Inf)};
comparison = struct('name', {}, 'level', {}, 'lapTime_s', {}, 'result', {});
for index = 1:numel(configs)
    result = lts.design.runCase(configs{index}, track);
    comparison(index).name = result.meta.modelName;
    comparison(index).level = result.meta.level;
    comparison(index).lapTime_s = result.lap.lapTime_s;
    comparison(index).result = result;
    fprintf('%-42s %9.3f s\n', result.meta.modelName, ...
        result.lap.lapTime_s);
end
resultDirectory = fullfile(projectRoot, 'results');
if ~isfolder(resultDirectory)
    mkdir(resultDirectory);
end
resultFile = fullfile(resultDirectory, 'slipModelComparison.mat');
assert(~isfile(resultFile), 'lts:io:ResultExists', ...
    'Result file already exists; refusing to overwrite: %s', resultFile);
save(resultFile, 'comparison', 'performanceMap');
end
