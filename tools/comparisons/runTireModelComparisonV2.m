function comparison = runTireModelComparisonV2(options)
%RUNTIREMODELCOMPARISONV2 Compare fitted and TMeasy tires through V2.
%   One compatible dualTrack slip map is built for each selected tire model.
%   Map resolution is configurable through the named input arguments.
arguments
    options.speedGrid_mps (1,:) double = [5,10,15,20,25,30]
    options.lateralFractions (1,:) double = [0,.25,.5,.75,1]
    options.boundaryIterations (1,1) double = 10
    options.interpolationSafetyFactor (1,1) double = .8
    options.showProgress (1,1) logical = true
end
%% Shared track and numerical settings
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root,fullfile(root,'vehicles'),fullfile(root,'config')); 
tracks=lts.io.trackCatalog(); track=lts.io.loadTrackMatrix(tracks.endurance,true);
tires=["fittedCombinedSlip","tmeasyAdtm"];
tl=lts.v2.trackLocalSettings(SpeedRows=3,LateralFractions=[], ...
    BoundaryIterations=6,InterpolationSafetyFactor=NaN,ShowProgress=false,AuditPointCount=8);
%% Build and evaluate one map per tire model
comparison=struct('tireModel',{},'lapTime_s',{},'energy',{},'map',{},'result',{});
for k=1:2
    vehicle=lts.tire.selectModel(FAUmaxTauV2("fourMotorAWD"),tires(k));
    map=lts.model.dualTrackSlip.buildPerformanceMap(vehicle, ...
        speedGrid_mps=options.speedGrid_mps,lateralFractions=options.lateralFractions, ...
        boundaryIterations=options.boundaryIterations, ...
        interpolationSafetyFactor=options.interpolationSafetyFactor, ...
        showProgress=options.showProgress);
    config=lts.v2.explicitCase(Vehicle=vehicle,Track=track,Model="dualTrack", ...
        Aero=true,LoadTransfer=true,TireSlip=true,Energy=true,YawInertia=false, ...
        ChassisTorsion=false,Strategy="mapped",MaximumSpeed_mps=100, ...
        InitialSpeed_mps=0,FinalSpeed_mps=Inf,PerformanceMap=map,TrackLocal=tl);
    result=lts.v2.runCase(config); comparison(k)=struct('tireModel',char(tires(k)), ...
        'lapTime_s',result.lap.lapTime_s,'energy',result.energy,'map',map,'result',result);
end
end
