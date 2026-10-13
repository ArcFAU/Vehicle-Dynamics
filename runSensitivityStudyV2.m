function study = runSensitivityStudyV2(caseConfig)
%RUNSENSITIVITYSTUDYV2 Run an explicit, reproducible V2 parameter sweep.
%   STUDY = runSensitivityStudyV2(CASECONFIG) evaluates every configured
%   parameter combination across the selected events. CASECONFIG should be
%   created by sensitivityStudyCaseV2. The returned metadata records the
%   effective V2 model family and feature selection. Starting this file
%   directly uses the visible default setup below.

%% Default user setup for direct Run-button execution
if nargin == 0
    root=fileparts(mfilename('fullpath'));
    addpath(fullfile(root,'config'),fullfile(root,'vehicles'));
    caseConfig=sensitivityStudyCaseV2(Study="mass",Model="pointMass", ...
        Aero=true,LoadTransfer=false,Energy=true, ...
        PowertrainLayout="fourMotorAWD",TireModel="fittedCombinedSlip", ...
        IncludeDriverless=false,CoupleEnergy=false,CreatePlots=true);
end

%% Validate the complete user configuration
required={'model','features','baseVehicle','parameters','events','scoring', ...
    'includeDriverless','coupleEnergy','createPlots','name'};
for k=1:numel(required),assert(isfield(caseConfig,required{k}), ...
        'lts:v2:StudyConfig','Missing field: %s',required{k});end
features=caseConfig.features;
featureNames={'aero','loadTransfer','tireSlip','energy','yawInertia','chassisTorsion'};
for k=1:numel(featureNames)
    assert(isfield(features,featureNames{k}) ...
        && islogical(features.(featureNames{k})) ...
        && isscalar(features.(featureNames{k})), 'lts:v2:StudyConfig', ...
        'Feature %s must be an explicit scalar logical.',featureNames{k});
end
assert(~features.tireSlip,'lts:v2:UnsupportedFeature', ...
    'Sensitivity V2 currently supports direct models only (tireSlip=false).');
assert(~caseConfig.coupleEnergy || features.energy,'lts:v2:EnergyCoupling', ...
    'coupleEnergy=true requires features.energy=true.');

% Validate the selected family before starting a potentially long sweep.
lts.v2.buildDirectModel(Vehicle=caseConfig.baseVehicle, ...
    Model=string(caseConfig.model),Aero=features.aero, ...
    LoadTransfer=features.loadTransfer,Energy=features.energy, ...
    YawInertia=features.yawInertia,ChassisTorsion=features.chassisTorsion);

%% Build the model afresh for every varied vehicle
factory=@(vehicle) lts.v2.buildDirectModel(Vehicle=vehicle, ...
    Model=string(caseConfig.model),Aero=features.aero, ...
    LoadTransfer=features.loadTransfer,Energy=features.energy, ...
    YawInertia=features.yawInertia,ChassisTorsion=features.chassisTorsion);
study=lts.study.runParameterSweep(caseConfig.baseVehicle,caseConfig.parameters, ...
    caseConfig.events,factory,caseConfig.scoring, ...
    IncludeDriverless=caseConfig.includeDriverless,CoupleEnergy=caseConfig.coupleEnergy);
study.meta.v2Configuration=rmfield(caseConfig,{'events','scoring','parameters'});

%% Optional compact engineering plots
if caseConfig.createPlots
    study.figures.sensitivity=plotStudy(study,caseConfig.name);
else
    study.figures.sensitivity=gobjects(0);
end

fprintf('V2 sensitivity complete: %d cases, model=%s\n', ...
    numel(study.cases),caseConfig.model);
end

function figureHandle=plotStudy(study,studyName)
%PLOTSTUDY Show points and event-time changes around the baseline.
figureHandle=figure('Name',['V2 sensitivity: ',studyName]);
layout=tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
title(layout,strrep(studyName,'_','\_'));
x=study.parameterValues(:,1);
nexttile;
plot(x,study.totalPoints,'o-','LineWidth',1.4);
grid on; ylabel('Total points');
nexttile;
plot(x,study.relative.eventTime_percent,'LineWidth',1.3);
grid on; xlabel(study.parameters(1).name); ylabel('\Delta event time [%]');
legend(string(study.eventIds),'Location','best');
end
