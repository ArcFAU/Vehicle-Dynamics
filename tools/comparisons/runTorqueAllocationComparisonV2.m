function comparison = runTorqueAllocationComparisonV2(powertrainLayoutName)
%RUNTORQUEALLOCATIONCOMPARISONV2 Compare fixed and ideal force allocation.
%   COMPARISON = runTorqueAllocationComparisonV2(LAYOUT) evaluates both
%   allocation modes with identical dualTrack V2 physics and energy output.
arguments
    powertrainLayoutName (1,1) string = "fourMotorAWD"
end
%% Shared vehicle and track
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root,fullfile(root,'vehicles'),fullfile(root,'config')); 
tracks=lts.io.trackCatalog(); track=lts.io.loadTrackMatrix(tracks.endurance,true);
base=FAUmaxTauV2(powertrainLayoutName); vehicles={base,lts.control.applyIdealAllocation(base)};
names=["fixed","ideal"];
tl=lts.v2.trackLocalSettings(SpeedRows=3,LateralFractions=[], ...
    BoundaryIterations=6,InterpolationSafetyFactor=NaN,ShowProgress=false,AuditPointCount=8);
%% Run both allocation modes
comparison=struct('mode',{},'lapTime_s',{},'energy',{},'result',{});
for k=1:2
    config=lts.v2.explicitCase(Vehicle=vehicles{k},Track=track,Model="dualTrack", ...
        Aero=true,LoadTransfer=true,TireSlip=false,Energy=true, ...
        YawInertia=false,ChassisTorsion=false,Strategy="direct", ...
        MaximumSpeed_mps=100,InitialSpeed_mps=0,FinalSpeed_mps=Inf, ...
        PerformanceMap=[],TrackLocal=tl);
    result=lts.v2.runCase(config);
    comparison(k)=struct('mode',char(names(k)),'lapTime_s',result.lap.lapTime_s, ...
        'energy',result.energy,'result',result);
end
end
