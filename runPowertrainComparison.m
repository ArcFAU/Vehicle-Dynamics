function comparison = runPowertrainComparison()
%RUNPOWERTRAINCOMPARISON Compare layouts with otherwise identical inputs.

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot, fullfile(projectRoot, 'vehicles'));
trackFiles = lts.io.trackCatalog();
trackFile = trackFiles.endurance;
track = lts.io.loadTrackMatrix(trackFile, true);
layoutNames = ["singleFrontMotor", "singleRearMotor", ...
    "dualFrontMotors", "dualRearMotors", "dualMotorAWD", "fourMotorAWD"];
comparison = struct('layout', {}, 'lapTime_s', {}, 'result', {});

for index = 1:numel(layoutNames)
    vehicle = FAUmaxTauV2(layoutNames(index));
    config = directCase(vehicle, "dualTrackEnvelope", false);
    result = lts.design.runCase(config, track);
    comparison(index).layout = char(layoutNames(index));
    comparison(index).lapTime_s = result.lap.lapTime_s;
    comparison(index).result = result;
    fprintf('%-24s %9.3f s\n', layoutNames(index), result.lap.lapTime_s);
end

resultDirectory = fullfile(projectRoot, 'results');
if ~isfolder(resultDirectory)
    mkdir(resultDirectory);
end
resultFile = fullfile(resultDirectory, 'powertrainComparison.mat');
assert(~isfile(resultFile), 'lts:io:ResultExists', ...
    'Result file already exists; refusing to overwrite: %s', resultFile);
save(resultFile, 'comparison');
end

function config = directCase(vehicle, modelId, includeEnergy)
config = lts.config.createCase(Vehicle=vehicle, Model=modelId, ...
    Strategy="direct", IncludeEnergy=includeEnergy, ...
    IncludeYawInertia=false, MaximumSpeed_mps=100, ...
    InitialSpeed_mps=0, FinalSpeed_mps=Inf);
end
