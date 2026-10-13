function comparison = runChassisStiffnessSlipComparisonV2(stiffness_NmPerDeg)
%RUNCHASSISSTIFFNESSSLIPCOMPARISONV2 Compare one finite chassis with rigid.
%   COMPARISON = runChassisStiffnessSlipComparisonV2(STIFFNESS) builds
%   independent track-local dualTrack maps for a fair chassis comparison.
arguments
    stiffness_NmPerDeg (1,1) double {mustBePositive,mustBeFinite} = 2000
end
%% Build rigid and finite-stiffness vehicles
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root,fullfile(root,'vehicles'),fullfile(root,'config'));
base=FAUmaxTauV2("fourMotorAWD");rigid=lts.chassis.applyTorsionalStiffness(base,Inf);
finite=lts.chassis.applyTorsionalStiffness(base,stiffness_NmPerDeg/deg2rad(1));
vehicles={rigid,finite};tracks=lts.io.trackCatalog();
track=lts.io.loadTrackMatrix(tracks.endurance,true);
tl=lts.v2.trackLocalSettings(SpeedRows=3,LateralFractions=linspace(0,1,7), ...
    BoundaryIterations=8,InterpolationSafetyFactor=.8,ShowProgress=true,AuditPointCount=8);
%% Run both transparent V2 cases
comparison=struct('stiffness_NmPerDeg',{},'lapTime_s',{},'result',{});
for k=1:2
    config=lts.v2.explicitCase(Vehicle=vehicles{k},Track=track,Model="dualTrack", ...
        Aero=true,LoadTransfer=true,TireSlip=true,Energy=false,YawInertia=false, ...
        ChassisTorsion=k==2,Strategy="trackLocal",MaximumSpeed_mps=120/3.6, ...
        InitialSpeed_mps=0,FinalSpeed_mps=Inf,PerformanceMap=[],TrackLocal=tl);
    result=lts.v2.runCase(config); values=[Inf,stiffness_NmPerDeg];
    comparison(k)=struct('stiffness_NmPerDeg',values(k), ...
        'lapTime_s',result.lap.lapTime_s,'result',result);
end
end
