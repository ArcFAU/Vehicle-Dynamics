function comparison = runChassisStiffnessComparison()
%RUNCHASSISSTIFFNESSCOMPARISON Sweep chassis torsional stiffness scenarios.

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot, fullfile(projectRoot, 'vehicles'));
baseVehicle = FAUmaxTauV2("fourMotorAWD");
stiffness_NmPerDeg = [500, 1000, 2000, 4000, Inf];
stiffness_NmPerRad = stiffness_NmPerDeg / deg2rad(1);
trackFiles = lts.io.trackCatalog();
trackFile = trackFiles.endurance;
track = lts.io.loadTrackMatrix(trackFile, true);
comparison = struct('torsionalStiffness_NmPerDeg', {}, ...
    'torsionalStiffness_NmPerRad', {}, 'frontRollMomentFraction', {}, ...
    'lapTime_s', {}, 'result', {});
for index = 1:numel(stiffness_NmPerRad)
    [vehicle, coupling] = lts.chassis.applyTorsionalStiffness(...
        baseVehicle, stiffness_NmPerRad(index));
    config = lts.config.createCase(Vehicle=vehicle, ...
        Model="dualTrackEnvelope", Strategy="direct", IncludeEnergy=false, ...
        IncludeYawInertia=false, MaximumSpeed_mps=100, ...
        InitialSpeed_mps=0, FinalSpeed_mps=Inf);
    result = lts.design.runCase(config, track);
    comparison(index).torsionalStiffness_NmPerDeg = ...
        stiffness_NmPerDeg(index);
    comparison(index).torsionalStiffness_NmPerRad = ...
        stiffness_NmPerRad(index);
    comparison(index).frontRollMomentFraction = ...
        coupling.frontRollMomentFraction;
    comparison(index).lapTime_s = result.lap.lapTime_s;
    comparison(index).result = result;
    fprintf('%8g Nm/deg  front=%6.3f  %9.3f s\n', ...
        stiffness_NmPerDeg(index), coupling.frontRollMomentFraction, ...
        result.lap.lapTime_s);
end
resultDirectory = fullfile(projectRoot, 'results');
if ~isfolder(resultDirectory)
    mkdir(resultDirectory);
end
resultFile = fullfile(resultDirectory, 'chassisStiffnessComparison.mat');
assert(~isfile(resultFile), 'lts:io:ResultExists', ...
    'Result file already exists; refusing to overwrite: %s', resultFile);
save(resultFile, 'comparison');
end
