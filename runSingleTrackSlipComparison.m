function comparison = runSingleTrackSlipComparison(performanceMap, options)
%RUNSINGLETRACKSLIPCOMPARISON Compare Levels 1.2 and 1.3 on one track.

arguments
    performanceMap = []
    options.BuildMap (1, 1) logical
end

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot, fullfile(projectRoot, 'vehicles'));
vehicle = FAUmaxTauV2("fourMotorAWD");
trackFiles = lts.io.trackCatalog();
trackFile = trackFiles.endurance;
track = lts.io.loadTrackMatrix(trackFile, true);
assert(isempty(performanceMap) == options.BuildMap, ...
    'lts:runner:SingleTrackMapSelection', ...
    ['Provide either an existing performanceMap with BuildMap=false or ' ...
    'an empty map with BuildMap=true.']);
if options.BuildMap
    performanceMap = lts.model.singleTrackSlip.buildPerformanceMap(vehicle);
end
configs = {lts.config.createCase(Vehicle=vehicle, ...
    Model="singleTrackAero", Strategy="direct", IncludeEnergy=false, ...
    IncludeYawInertia=false, MaximumSpeed_mps=100, InitialSpeed_mps=0, ...
    FinalSpeed_mps=Inf), ...
    lts.config.createCase(Vehicle=vehicle, Model="singleTrackSlip", ...
    Strategy="mapped", PerformanceMap=performanceMap, IncludeEnergy=false, ...
    IncludeYawInertia=false, MaximumSpeed_mps=100, InitialSpeed_mps=0, ...
    FinalSpeed_mps=Inf)};
comparison = repmat(struct(), 1, numel(configs));
for index = 1:numel(configs)
    result = lts.design.runCase(configs{index}, track);
    comparison(index).name = result.meta.modelName;
    comparison(index).level = result.meta.level;
    comparison(index).lapTime_s = result.lap.lapTime_s;
    comparison(index).result = result;
    comparison(index).brakeDemand = result.brakeDemand;
    comparison(index).yawDemand = result.yawDemand;
    fprintf('%s: %.3f s\n', result.meta.modelName, result.lap.lapTime_s);
end
end
