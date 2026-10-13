function comparison = runPowertrainEnergyComparisonV2()
%RUNPOWERTRAINENERGYCOMPARISONV2 Compare event energy for all powertrains.
%   The FSG endurance event is evaluated with the same direct dualTrack V2
%   physics. Effective feature selections are stored with every result.

%% Event and layout selection
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root,fullfile(root,'vehicles'),fullfile(root,'config'));
events=lts.config.events.fsg2026(); endurance=events(strcmp({events.id},'endurance'));
layouts=["singleFrontMotor","singleRearMotor","dualFrontMotors", ...
    "dualRearMotors","dualMotorAWD","fourMotorAWD"];
%% Run endurance for every layout
comparison=struct('layout',{},'eventResult',{},'effectiveConfiguration',{});
for k=1:numel(layouts)
    vehicle=FAUmaxTauV2(layouts(k));
    [model,effective]=lts.v2.buildDirectModel(Vehicle=vehicle,Model="dualTrack", ...
        Aero=true,LoadTransfer=true,Energy=true,YawInertia=false,ChassisTorsion=false);
    eventResult=lts.events.runSet(endurance,model,IncludeDriverless=false,CoupleEnergy=false);
    comparison(k)=struct('layout',char(layouts(k)),'eventResult',eventResult, ...
        'effectiveConfiguration',effective);
end
%% Save to a dedicated V2 result file
folder=fullfile(root,'results');
file=lts.v2.nextResultFile(string(folder),"powertrainEnergyComparisonV2");
save(file,'comparison');
end
