function comparison = runTorqueAllocationComparison(powertrainLayoutName)
%RUNTORQUEALLOCATIONCOMPARISON Compare fixed and ideal wheel allocation.

arguments
    powertrainLayoutName (1, 1) string
end

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot, fullfile(projectRoot, 'vehicles'));
trackFiles = lts.io.trackCatalog();
trackFile = trackFiles.endurance;
track = lts.io.loadTrackMatrix(trackFile, true);

fixedVehicle = FAUmaxTauV2(powertrainLayoutName);
idealVehicle = lts.control.applyIdealAllocation(fixedVehicle);
vehicles = {fixedVehicle, idealVehicle};
modeNames = ["fixed", "ideal"];
comparison = struct('mode', {}, 'lapTime_s', {}, ...
    'netBatteryEnergy_kWh', {}, 'result', {}, 'energy', {});

for index = 1:numel(vehicles)
    vehicle = vehicles{index};
    result = lts.design.runCase(directCase(vehicle, true), track);
    energy = result.energy;
    comparison(index).mode = char(modeNames(index));
    comparison(index).lapTime_s = result.lap.lapTime_s;
    comparison(index).netBatteryEnergy_kWh = ...
        energy.netBatteryEnergy_kWh;
    comparison(index).result = result;
    comparison(index).energy = energy;
    fprintf('%-8s %9.3f s  %8.4f kWh\n', modeNames(index), ...
        result.lap.lapTime_s, energy.netBatteryEnergy_kWh);
end

comparison(2).lapTimePotential_s = ...
    comparison(1).lapTime_s - comparison(2).lapTime_s;
comparison(2).lapTimePotential_percent = 100 ...
    * comparison(2).lapTimePotential_s / comparison(1).lapTime_s;

resultDirectory = fullfile(projectRoot, 'results');
if ~isfolder(resultDirectory)
    mkdir(resultDirectory);
end
resultFile = fullfile(resultDirectory, 'torqueAllocationComparison.mat');
assert(~isfile(resultFile), 'lts:io:ResultExists', ...
    'Result file already exists; refusing to overwrite: %s', resultFile);
save(resultFile, 'comparison');
end

function config = directCase(vehicle, includeEnergy)
config = lts.config.createCase(Vehicle=vehicle, Model="dualTrackEnvelope", ...
    Strategy="direct", IncludeEnergy=includeEnergy, ...
    IncludeYawInertia=false, MaximumSpeed_mps=100, ...
    InitialSpeed_mps=0, FinalSpeed_mps=Inf);
end
