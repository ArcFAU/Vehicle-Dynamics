function comparison = runChassisStiffnessSlipComparison(torsionalStiffness_NmPerDeg)
%RUNCHASSISSTIFFNESSSLIPCOMPARISON Compare one finite chassis with rigid.

arguments
    torsionalStiffness_NmPerDeg (1, 1) double ...
        {mustBePositive, mustBeFinite}
end

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot, fullfile(projectRoot, 'vehicles'));
resultDirectory = fullfile(projectRoot, 'results');
rigidFile = fullfile(resultDirectory, 'rollStiffnessSlipComparison.mat');
assert(isfile(rigidFile), 'lts:runner:MissingRigidSlipMap', ...
    'Run runRollStiffnessSlipComparison before this comparison.');
rigidData = load(rigidFile, 'derivedMap');
rigidMap = rigidData.derivedMap;
baseVehicle = FAUmaxTauV2("fourMotorAWD");
[rigidVehicle, ~] = lts.chassis.applyTorsionalStiffness(baseVehicle, Inf);
finiteStiffness_NmPerRad = torsionalStiffness_NmPerDeg / deg2rad(1);
[finiteVehicle, coupling] = lts.chassis.applyTorsionalStiffness(...
    baseVehicle, finiteStiffness_NmPerRad);
finiteMap = lts.model.dualTrackSlip.buildPerformanceMap(finiteVehicle, ...
    speedGrid_mps=rigidMap.speedGrid_mps, ...
    lateralFractions=rigidMap.lateralFractions, ...
    boundaryIterations=rigidMap.meta.boundaryIterations, ...
    interpolationSafetyFactor=rigidMap.meta.interpolationSafetyFactor);
trackFiles = lts.io.trackCatalog();
trackFile = trackFiles.endurance;
track = lts.io.loadTrackMatrix(trackFile, true);
vehicles = {rigidVehicle, finiteVehicle};
maps = {rigidMap, finiteMap};
names = {'Rigid chassis slip map', ...
    sprintf('%g Nm/deg chassis slip map', torsionalStiffness_NmPerDeg)};
comparison = struct('name', {}, 'torsionalStiffness_NmPerDeg', {}, ...
    'frontRollMomentFraction', {}, 'lapTime_s', {}, 'result', {});
stiffnessValues = [Inf, torsionalStiffness_NmPerDeg];
for index = 1:numel(vehicles)
    config = lts.config.createCase(Model="dualTrackSlipFitted", ...
        Vehicle=vehicles{index}, Strategy="mapped", ...
        PerformanceMap=maps{index}, IncludeEnergy=false, ...
        IncludeYawInertia=false, MaximumSpeed_mps=Inf, ...
        InitialSpeed_mps=0, FinalSpeed_mps=0);
    result = lts.design.runCase(config, track);
    comparison(index).name = names{index};
    comparison(index).torsionalStiffness_NmPerDeg = stiffnessValues(index);
    comparison(index).frontRollMomentFraction = ...
        vehicles{index}.suspension.frontRollMomentFraction;
    comparison(index).lapTime_s = result.lap.lapTime_s;
    comparison(index).result = result;
    fprintf('%-34s front=%6.3f  %9.3f s\n', names{index}, ...
        comparison(index).frontRollMomentFraction, result.lap.lapTime_s);
end
fileName = sprintf('chassisStiffnessSlipComparison_%gNmPerDeg.mat', ...
    torsionalStiffness_NmPerDeg);
resultFile = fullfile(resultDirectory, fileName);
assert(~isfile(resultFile), 'lts:io:ResultExists', ...
    'Result file already exists; refusing to overwrite: %s', resultFile);
save(resultFile, 'comparison', 'rigidMap', 'finiteMap', 'coupling');
end
