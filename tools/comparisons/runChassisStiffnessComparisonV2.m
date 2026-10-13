function comparison = runChassisStiffnessComparisonV2()
%RUNCHASSISSTIFFNESSCOMPARISONV2 Sweep chassis stiffness using dualTrack V2.
%   Each stiffness receives its own track-local slip map. Finite stiffness
%   explicitly enables chassisTorsion; the rigid case disables the feature.

%% Sweep definition and common track
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root,fullfile(root,'vehicles'),fullfile(root,'config'));
base=FAUmaxTauV2("fourMotorAWD"); values=[500,1000,2000,4000,Inf];
tracks=lts.io.trackCatalog();track=lts.io.loadTrackMatrix(tracks.endurance,true);
tl=lts.v2.trackLocalSettings(SpeedRows=3,LateralFractions=[],BoundaryIterations=6, ...
    InterpolationSafetyFactor=.8,ShowProgress=true,AuditPointCount=8);
%% Run every stiffness as an independent transparent V2 case
comparison=struct('torsionalStiffness_NmPerDeg',{},'lapTime_s',{},'result',{});
for k=1:numel(values)
    vehicle=lts.chassis.applyTorsionalStiffness(base,values(k)/deg2rad(1));
    config=lts.v2.explicitCase(Vehicle=vehicle,Track=track,Model="dualTrack", ...
        Aero=true,LoadTransfer=true,TireSlip=true,Energy=false,YawInertia=false, ...
        ChassisTorsion=isfinite(values(k)),Strategy="trackLocal", ...
        MaximumSpeed_mps=120/3.6,InitialSpeed_mps=0,FinalSpeed_mps=Inf, ...
        PerformanceMap=[],TrackLocal=tl);
    result=lts.v2.runCase(config);comparison(k)=struct( ...
        'torsionalStiffness_NmPerDeg',values(k),'lapTime_s',result.lap.lapTime_s, ...
        'result',result);
end
end
