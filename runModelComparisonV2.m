function comparison = runModelComparisonV2(modelFamilies)
%RUNMODELCOMPARISONV2 Compare the three transparent V2 model families.
%   COMPARISON = runModelComparisonV2(FAMILIES) evaluates the requested
%   pointMass, singleTrack and dualTrack families on the same endurance
%   track. Physical features and solver settings are identical and explicit.
%
%   Example:
%       comparison = runModelComparisonV2(...
%           ["pointMass", "singleTrack", "dualTrack"]);
arguments
    modelFamilies (1, :) string = ["pointMass", "singleTrack", "dualTrack"]
end
assert(all(ismember(modelFamilies, ["pointMass", "singleTrack", "dualTrack"])), ...
    'lts:v2:RunnerModel', 'Use only pointMass, singleTrack or dualTrack.');
%% Shared vehicle, track and numerical settings
root = fileparts(mfilename('fullpath')); addpath(root, fullfile(root,'vehicles'));
vehicle = FAUmaxTauV2("fourMotorAWD"); tracks = lts.io.trackCatalog();
track = lts.io.loadTrackMatrix(tracks.endurance, true);
tl = lts.v2.trackLocalSettings(SpeedRows=3, LateralFractions=[], ...
    BoundaryIterations=6, InterpolationSafetyFactor=NaN, ...
    ShowProgress=false, AuditPointCount=8);
%% Run requested model families
comparison = struct('model', {}, 'lapTime_s', {}, 'result', {});
for k = 1:numel(modelFamilies)
    family = modelFamilies(k);
    config = lts.v2.explicitCase(Vehicle=vehicle, Track=track, Model=family, ...
        Aero=true, LoadTransfer=family ~= "pointMass", TireSlip=false, ...
        Energy=false, YawInertia=false, ChassisTorsion=false, ...
        Strategy="direct", MaximumSpeed_mps=100, InitialSpeed_mps=0, ...
        FinalSpeed_mps=Inf, PerformanceMap=[], TrackLocal=tl);
    result = lts.v2.runCase(config);
    comparison(k) = struct('model', char(family), ...
        'lapTime_s', result.lap.lapTime_s, 'result', result);
end
end
