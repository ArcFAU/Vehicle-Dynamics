function comparison = runRollStiffnessComparisonV2()
%RUNROLLSTIFFNESSCOMPARISONV2 Compare configured and derived roll balance.
%   Both vehicles are evaluated with the direct dualTrack V2 family. The
%   derived elastic properties and results are stored in a dedicated V2 file.

%% Build configured and suspension-derived vehicles
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root,fullfile(root,'vehicles'),fullfile(root,'config')); 
base=FAUmaxTauV2("fourMotorAWD"); [derived,properties]=lts.suspension.applyRollStiffness(base);
vehicles={base,derived}; names=["configured","derived"];
tracks=lts.io.trackCatalog(); track=lts.io.loadTrackMatrix(tracks.endurance,true);
tl=lts.v2.trackLocalSettings(SpeedRows=3,LateralFractions=[],BoundaryIterations=6, ...
    InterpolationSafetyFactor=NaN,ShowProgress=false,AuditPointCount=8);
%% Run both roll distributions
comparison=struct('name',{},'frontRollMomentFraction',{},'lapTime_s',{},'result',{});
for k=1:2
    config=lts.v2.explicitCase(Vehicle=vehicles{k},Track=track,Model="dualTrack", ...
        Aero=true,LoadTransfer=true,TireSlip=false,Energy=false,YawInertia=false, ...
        ChassisTorsion=false,Strategy="direct",MaximumSpeed_mps=120/3.6, ...
        InitialSpeed_mps=0,FinalSpeed_mps=Inf,PerformanceMap=[],TrackLocal=tl);
    result=lts.v2.runCase(config); comparison(k)=struct('name',char(names(k)), ...
        'frontRollMomentFraction',vehicles{k}.suspension.frontRollMomentFraction, ...
        'lapTime_s',result.lap.lapTime_s,'result',result);
end
folder=fullfile(root,'results');
file=lts.v2.nextResultFile(string(folder),"rollStiffnessComparisonV2");
save(file,'comparison','properties');
end
