function result = runDemo()
%RUNDEMO Run the design-ready Level 0.3 workflow on Endurance.

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot, fullfile(projectRoot, 'vehicles'));

vehicle = FAUmaxTauV2("fourMotorAWD");
trackFiles = lts.io.trackCatalog();
track = lts.io.loadTrackMatrix(trackFiles.endurance, true);
config = lts.config.createCase(Vehicle=vehicle, Model="pointMassAero", ...
    Strategy="direct", IncludeEnergy=true, IncludeYawInertia=false, ...
    MaximumSpeed_mps=100, InitialSpeed_mps=0, FinalSpeed_mps=Inf);
result = lts.design.runCase(config, track);

fprintf('Model: %s\n', result.meta.modelName);
fprintf('Track: %s\n', track.meta.name);
fprintf('Lap time: %.3f s\n', result.lap.lapTime_s);
fprintf('Solver iterations: %d\n', result.lap.meta.iterations);

figure('Name', 'nextGen V2 Level 0.3');
plot(result.lap.distance_m, result.lap.speed_mps * 3.6, 'LineWidth', 1.5);
grid on;
xlabel('Distance [m]');
ylabel('Speed [km/h]');
title('Level 0.3 Point-Mass Speed Profile');
end
