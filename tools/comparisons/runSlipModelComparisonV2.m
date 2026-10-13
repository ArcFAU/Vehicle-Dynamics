function comparison = runSlipModelComparisonV2()
%RUNSLIPMODELCOMPARISONV2 Compare direct and mapped dualTrack V2 physics.
%   Builds a reusable fitted-tire performance map, runs both configurations
%   on endurance and saves results as slipModelComparisonV2.mat.

%% Vehicle, track and reusable slip map
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root,fullfile(root,'vehicles'),fullfile(root,'config')); 
vehicle=FAUmaxTauV2("fourMotorAWD"); tracks=lts.io.trackCatalog();
track=lts.io.loadTrackMatrix(tracks.endurance,true);
map=lts.model.dualTrackSlip.buildPerformanceMap(vehicle, ...
    lateralFractions=linspace(0,1,7),boundaryIterations=12, ...
    maximumAccelerationSearch_mps2=5*9.81,interpolationSafetyFactor=1, ...
    showProgress=true,progressLabel="V2 slip comparison",enableSolverRetries=true);
tl=lts.v2.trackLocalSettings(SpeedRows=3,LateralFractions=[], ...
    BoundaryIterations=6,InterpolationSafetyFactor=NaN,ShowProgress=false,AuditPointCount=8);
%% Explicit direct and mapped configurations
configs={make(false,[],"direct"),make(true,map,"mapped")};
comparison=struct('tireSlip',{},'lapTime_s',{},'result',{});
for k=1:2
    result=lts.v2.runCase(configs{k}); comparison(k)=struct( ...
        'tireSlip',k==2,'lapTime_s',result.lap.lapTime_s,'result',result);
end
%% Save without touching legacy comparison data
folder=fullfile(root,'results');
file=lts.v2.nextResultFile(string(folder),"slipModelComparisonV2");
performanceMap=map; save(file,'comparison','performanceMap');
    function config=make(slip,performanceMap,strategy)
        config=lts.v2.explicitCase(Vehicle=vehicle,Track=track,Model="dualTrack", ...
            Aero=true,LoadTransfer=true,TireSlip=slip,Energy=false, ...
            YawInertia=false,ChassisTorsion=false,Strategy=strategy, ...
            MaximumSpeed_mps=120/3.6,InitialSpeed_mps=0,FinalSpeed_mps=Inf, ...
            PerformanceMap=performanceMap,TrackLocal=tl);
    end
end
