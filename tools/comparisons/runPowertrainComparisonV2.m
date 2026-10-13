function comparison = runPowertrainComparisonV2()
%RUNPOWERTRAINCOMPARISONV2 Compare powertrain layouts with explicit V2 physics.
%   All layouts use the same dualTrack configuration. Results are returned
%   and saved separately as results/powertrainComparisonV2.mat.

%% Shared track and explicit model settings
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root,fullfile(root,'vehicles'),fullfile(root,'config')); 
tracks=lts.io.trackCatalog(); track=lts.io.loadTrackMatrix(tracks.endurance,true);
layouts=["singleFrontMotor","singleRearMotor","dualFrontMotors", ...
    "dualRearMotors","dualMotorAWD","fourMotorAWD"];
tl=lts.v2.trackLocalSettings(SpeedRows=3,LateralFractions=[], ...
    BoundaryIterations=6,InterpolationSafetyFactor=NaN, ...
    ShowProgress=false,AuditPointCount=8);
%% Simulate every powertrain layout
comparison=struct('layout',{},'lapTime_s',{},'result',{});
for k=1:numel(layouts)
    vehicle=FAUmaxTauV2(layouts(k));
    config=lts.v2.explicitCase(Vehicle=vehicle,Track=track,Model="dualTrack", ...
        Aero=true,LoadTransfer=true,TireSlip=false,Energy=false, ...
        YawInertia=false,ChassisTorsion=false,Strategy="direct", ...
        MaximumSpeed_mps=100,InitialSpeed_mps=0,FinalSpeed_mps=Inf, ...
        PerformanceMap=[],TrackLocal=tl);
    result=lts.v2.runCase(config);
    comparison(k)=struct('layout',char(layouts(k)), ...
        'lapTime_s',result.lap.lapTime_s,'result',result);
end
%% Save without overwriting an existing V2 result
saveNew(root,'powertrainComparisonV2.mat',comparison);
end
function saveNew(root,name,comparison)
folder=fullfile(root,'results'); [~,baseName]=fileparts(name);
file=lts.v2.nextResultFile(string(folder),string(baseName));
save(file,'comparison');
end
