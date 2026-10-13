function comparison = runRollStiffnessSlipComparisonV2()
%RUNROLLSTIFFNESSSLIPCOMPARISONV2 Compare roll balance with local slip maps.
%   This detailed workflow regenerates a track-local map for each vehicle,
%   preventing reuse of a map with incompatible suspension properties.

%% Prepare the two vehicle definitions
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root,fullfile(root,'vehicles'),fullfile(root,'config'));
base=FAUmaxTauV2("fourMotorAWD");derived=lts.suspension.applyRollStiffness(base);
vehicles={base,derived};tracks=lts.io.trackCatalog();
track=lts.io.loadTrackMatrix(tracks.endurance,true);
tl=lts.v2.trackLocalSettings(SpeedRows=3,LateralFractions=linspace(0,1,7), ...
    BoundaryIterations=8,InterpolationSafetyFactor=.8,ShowProgress=true,AuditPointCount=8);
%% Build and run a local map for each case
comparison=struct('frontRollMomentFraction',{},'lapTime_s',{},'result',{});
for k=1:2
    config=lts.v2.explicitCase(Vehicle=vehicles{k},Track=track,Model="dualTrack", ...
        Aero=true,LoadTransfer=true,TireSlip=true,Energy=false,YawInertia=false, ...
        ChassisTorsion=false,Strategy="trackLocal",MaximumSpeed_mps=120/3.6, ...
        InitialSpeed_mps=0,FinalSpeed_mps=Inf,PerformanceMap=[],TrackLocal=tl);
    result=lts.v2.runCase(config);comparison(k)=struct( ...
        'frontRollMomentFraction',vehicles{k}.suspension.frontRollMomentFraction, ...
        'lapTime_s',result.lap.lapTime_s,'result',result);
end
end
