function comparison = runRollStiffnessComparison()
%RUNROLLSTIFFNESSCOMPARISON Compare fixed and derived roll distributions.

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot, fullfile(projectRoot, 'vehicles'));
baselineVehicle = FAUmaxTauV2("fourMotorAWD");
[derivedVehicle, rollProperties] = ...
    lts.suspension.applyRollStiffness(baselineVehicle);
trackFiles = lts.io.trackCatalog();
trackFile = trackFiles.endurance;
track = lts.io.loadTrackMatrix(trackFile, true);
vehicles = {baselineVehicle, derivedVehicle};
names = {'Fixed 50/50 roll moment', 'Derived elastic roll stiffness'};
comparison = struct('name', {}, 'frontRollMomentFraction', {}, ...
    'lapTime_s', {}, 'result', {});
for index = 1:numel(vehicles)
    config = lts.config.createCase(Vehicle=vehicles{index}, ...
        Model="dualTrackEnvelope", Strategy="direct", IncludeEnergy=false, ...
        IncludeYawInertia=false, MaximumSpeed_mps=100, ...
        InitialSpeed_mps=0, FinalSpeed_mps=Inf);
    result = lts.design.runCase(config, track);
    comparison(index).name = names{index};
    comparison(index).frontRollMomentFraction = ...
        vehicles{index}.suspension.frontRollMomentFraction;
    comparison(index).lapTime_s = result.lap.lapTime_s;
    comparison(index).result = result;
    fprintf('%-36s front=%6.3f  %9.3f s\n', names{index}, ...
        comparison(index).frontRollMomentFraction, result.lap.lapTime_s);
end
resultDirectory = fullfile(projectRoot, 'results');
if ~isfolder(resultDirectory)
    mkdir(resultDirectory);
end
resultFile = fullfile(resultDirectory, 'rollStiffnessComparison.mat');
assert(~isfile(resultFile), 'lts:io:ResultExists', ...
    'Result file already exists; refusing to overwrite: %s', resultFile);
save(resultFile, 'comparison', 'rollProperties');
end
