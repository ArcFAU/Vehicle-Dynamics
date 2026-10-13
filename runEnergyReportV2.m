function report = runEnergyReportV2(caseConfig)
%RUNENERGYREPORTV2 Evaluate and print the V2 baseline energy targets.
%   REPORT = runEnergyReportV2(CASECONFIG) reduces a V2 sensitivity study
%   to its baseline values, evaluates all configured events and returns the
%   complete event-energy data. CASECONFIG must enable the energy feature.
%   Starting this file directly uses the visible baseline setup below.

%% Default user setup for direct Run-button execution
if nargin == 0
    root=fileparts(mfilename('fullpath'));
    addpath(fullfile(root,'config'),fullfile(root,'vehicles'));
    caseConfig=sensitivityStudyCaseV2(Study="mass",Model="pointMass", ...
        Aero=true,LoadTransfer=false,Energy=true, ...
        PowertrainLayout="fourMotorAWD",TireModel="fittedCombinedSlip", ...
        IncludeDriverless=false,CoupleEnergy=false,CreatePlots=false);
end

%% Validate and reduce the sweep to its baseline
assert(isstruct(caseConfig),'lts:v2:StudyConfig', ...
    'Pass a sensitivityStudyCaseV2 configuration.');
assert(isfield(caseConfig,'features')&&caseConfig.features.energy, ...
    'lts:v2:EnergyDisabled','Energy report requires features.energy=true.');
baseline=caseConfig.parameters(1);baseline.values=baseline.baseline;
caseConfig.parameters=baseline;study=runSensitivityStudyV2(caseConfig);

%% Assemble and print the report
report.meta=study.meta;report.vehicle=study.cases(1).vehicle;
report.events=study.cases(1).eventResults;
fprintf('\n%-24s %10s %12s\n','Event','Time [s]','Energy [kWh]');
for k=1:numel(report.events)
    fprintf('%-24s %10.3f %12.4f\n',report.events(k).name, ...
        report.events(k).time_s,report.events(k).energy.netBatteryEnergy_kWh);
end
idx=find(strcmp({report.events.id},'endurance'),1);
if ~isempty(idx),report.accumulatorTarget=report.events(idx).energy;end
end
